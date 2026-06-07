from __future__ import annotations

import threading
import time
from dataclasses import dataclass, field
from typing import Optional

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import Twist

try:
    import serial
except ImportError as exc:  # pragma: no cover - runtime dependency on robot
    serial = None

from techx_r2_chassis_interfaces.action import LiftControl, StepCommand
from techx_r2_chassis_interfaces.msg import LiftStatus as LiftStatusMsg
from techx_r2_chassis_interfaces.srv import EStop

from .protocol import (
    CMD_LIFT_STATUS,
    CMD_STEP_STATUS,
    ERROR_MOTOR_ERROR,
    ERROR_OK,
    FRAME_LEN,
    STATE_ACK,
    STATE_DONE,
    STATE_ERROR,
    STATE_NAME,
    STATE_RUNNING,
    build_chassis_vel_cmd,
    build_estop,
    build_lift_control,
    build_step_command,
    extract_frames,
    frame_to_hex,
    parse_edge_id,
    parse_lift_status,
    parse_step_status,
    step_cmd_from_string,
)

MODE_NORMAL_VELOCITY = "NormalVelocity"
MODE_TASK = "TaskMode"

TASK_KIND_STEP = "step"
TASK_KIND_LIFT = "lift"


@dataclass
class LatestVelocity:
    vx_mm_s: int = 0
    vy_mm_s: int = 0
    omega_mrad_s: int = 0
    last_time_sec: float = 0.0


@dataclass
class PendingTask:
    """A generic reliable task.

    First version keeps only one reliable task at a time.  The task can be a
    STEP_COMMAND or a LIFT_CONTROL command.  ACK timeout / resend / final result
    handling is shared by both task kinds.
    """

    kind: str
    seq: int
    frame: bytes
    task_id: int
    cmd_text: str
    timeout_sec: float
    goal_handle: object
    created_time: float = field(default_factory=time.monotonic)
    last_send_time: float = field(default_factory=lambda: 0.0)
    retry_count: int = 0
    ack_received: bool = False
    final_state: int = 0
    error_code: int = ERROR_OK
    message: str = ""
    done_event: threading.Event = field(default_factory=threading.Event)


class ChassisSerialNode(Node):
    def __init__(self) -> None:
        super().__init__("chassis_serial_node")

        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("serial_timeout_sec", 0.02)
        self.declare_parameter("velocity_rate_hz", 30.0)
        self.declare_parameter("cmd_vel_timeout_sec", 0.2)
        self.declare_parameter("task_ack_timeout_sec", 0.1)
        self.declare_parameter("task_max_retries", 3)
        self.declare_parameter("task_default_timeout_sec", 10.0)
        self.declare_parameter("send_zero_in_task_mode", True)
        self.declare_parameter("log_velocity_frames", False)
        self.declare_parameter("log_rx_frames", False)
        self.declare_parameter("estop_repeat_count", 3)

        self.port = self.get_parameter("port").value
        self.baudrate = int(self.get_parameter("baudrate").value)
        serial_timeout_sec = float(self.get_parameter("serial_timeout_sec").value)

        self._serial = None
        if serial is None:
            raise RuntimeError("pyserial is not installed. Install with: sudo apt install python3-serial")
        self._serial = serial.Serial(self.port, self.baudrate, timeout=serial_timeout_sec)
        self.get_logger().info(f"Opened chassis serial port {self.port} @ {self.baudrate}")

        self._write_lock = threading.Lock()
        self._state_lock = threading.RLock()
        self._rx_buffer = bytearray()
        self._stop_event = threading.Event()

        self._mode = MODE_NORMAL_VELOCITY
        self._latest_velocity = LatestVelocity(last_time_sec=0.0)
        self._vel_seq = 0
        self._task_seq = 1
        self._pending_task: Optional[PendingTask] = None

        self._cmd_vel_sub = self.create_subscription(Twist, "/cmd_vel", self._on_cmd_vel, 10)

        self._estop_srv = self.create_service(EStop, "/r2_chassis/estop", self._on_estop)

        self._step_action_server = ActionServer(
            self,
            StepCommand,
            "/r2_chassis/step_command",
            execute_callback=self._execute_step_command,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
        )

        self._lift_status_pub = self.create_publisher(
            LiftStatusMsg,
            "/r2_chassis/lift_status",
            10,
        )

        self._lift_action_server = ActionServer(
            self,
            LiftControl,
            "/r2_chassis/lift_control",
            execute_callback=self._execute_lift_control,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
        )

        velocity_rate_hz = float(self.get_parameter("velocity_rate_hz").value)
        self._velocity_timer = self.create_timer(1.0 / max(1.0, velocity_rate_hz), self._velocity_timer_cb)
        self._reliable_timer = self.create_timer(0.02, self._reliable_timer_cb)

        self._rx_thread = threading.Thread(target=self._serial_read_loop, daemon=True)
        self._rx_thread.start()

    def destroy_node(self) -> bool:
        self._stop_event.set()
        try:
            if self._serial is not None and self._serial.is_open:
                self._serial.close()
        except Exception:
            pass
        return super().destroy_node()

    def _now_monotonic(self) -> float:
        return time.monotonic()

    def _next_task_seq(self) -> int:
        with self._state_lock:
            seq = self._task_seq & 0xFF
            self._task_seq = (self._task_seq + 1) & 0xFF
            if self._task_seq == 0:
                self._task_seq = 1
            return seq

    def _write_frame(self, frame: bytes, reason: str = "") -> None:
        with self._write_lock:
            self._serial.write(frame)
            self._serial.flush()
        if reason:
            self.get_logger().debug(f"TX {reason}: {frame_to_hex(frame)}")

    def _send_zero_velocity(self) -> None:
        self._vel_seq = (self._vel_seq + 1) & 0xFF
        frame = build_chassis_vel_cmd(self._vel_seq, 0, 0, 0)
        self._write_frame(frame, "zero_vel")

    def _on_cmd_vel(self, msg: Twist) -> None:
        with self._state_lock:
            self._latest_velocity = LatestVelocity(
                vx_mm_s=int(round(msg.linear.x * 1000.0)),
                vy_mm_s=int(round(msg.linear.y * 1000.0)),
                omega_mrad_s=int(round(msg.angular.z * 1000.0)),
                last_time_sec=self._now_monotonic(),
            )

    def _velocity_timer_cb(self) -> None:
        send_zero_in_task = bool(self.get_parameter("send_zero_in_task_mode").value)
        cmd_vel_timeout_sec = float(self.get_parameter("cmd_vel_timeout_sec").value)
        log_velocity_frames = bool(self.get_parameter("log_velocity_frames").value)

        with self._state_lock:
            mode = self._mode
            latest = self._latest_velocity
            stale = latest.last_time_sec <= 0.0 or (self._now_monotonic() - latest.last_time_sec) > cmd_vel_timeout_sec
            self._vel_seq = (self._vel_seq + 1) & 0xFF
            seq = self._vel_seq

        if mode != MODE_NORMAL_VELOCITY:
            if not send_zero_in_task:
                return
            frame = build_chassis_vel_cmd(seq, 0, 0, 0)
            self._write_frame(frame, "task_zero_vel" if log_velocity_frames else "")
            return

        if stale:
            frame = build_chassis_vel_cmd(seq, 0, 0, 0)
        else:
            frame = build_chassis_vel_cmd(seq, latest.vx_mm_s, latest.vy_mm_s, latest.omega_mrad_s)
        self._write_frame(frame, "vel" if log_velocity_frames else "")

    def _goal_callback(self, goal_request) -> GoalResponse:
        del goal_request
        with self._state_lock:
            if self._pending_task is not None:
                self.get_logger().warn("Reject reliable task goal: another reliable task is running")
                return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle) -> CancelResponse:
        del goal_handle
        self.get_logger().warn("Cancel requested, but first version does not cancel STM32 task; use ESTOP if needed")
        return CancelResponse.REJECT

    def _execute_step_command(self, goal_handle) -> StepCommand.Result:
        goal = goal_handle.request
        result = StepCommand.Result()

        try:
            step_cmd = step_cmd_from_string(goal.cmd_type)
        except ValueError as exc:
            goal_handle.abort()
            result.success = False
            result.final_state = STATE_ERROR
            result.error_code = ERROR_MOTOR_ERROR
            result.message = str(exc)
            return result

        timeout_sec = float(goal.timeout_sec) if goal.timeout_sec and goal.timeout_sec > 0 else float(
            self.get_parameter("task_default_timeout_sec").value
        )

        # 第一版 STEP_COMMAND 只使用 step_cmd。
        # delta_h_mm / edge_id / flags 暂时不参与 STM32 控制，统一填 0。
        delta_h_mm = 0
        edge_id = 0
        flags = 0

        seq = self._next_task_seq()
        frame = build_step_command(seq, step_cmd, delta_h_mm, edge_id=edge_id, flags=flags)

        pending = PendingTask(
            kind=TASK_KIND_STEP,
            seq=seq,
            frame=frame,
            task_id=step_cmd,
            cmd_text=goal.cmd_type,
            timeout_sec=timeout_sec,
            goal_handle=goal_handle,
        )

        with self._state_lock:
            if self._pending_task is not None:
                goal_handle.abort()
                result.success = False
                result.final_state = STATE_ERROR
                result.error_code = ERROR_MOTOR_ERROR
                result.message = "another reliable task is running"
                return result
            self._mode = MODE_TASK
            self._pending_task = pending

        self.get_logger().info(
            f"Start STEP_COMMAND seq={seq} cmd={goal.cmd_type} "
            f"delta_h=0 edge_id=0 flags=0 timeout={timeout_sec:.2f}s"
        )
        self._send_zero_velocity()
        pending.last_send_time = self._now_monotonic()
        self._write_frame(frame, "step_command")

        pending.done_event.wait(timeout=timeout_sec + 1.0)

        with self._state_lock:
            final_state = pending.final_state
            error_code = pending.error_code
            message = pending.message or "step_command ended without final message"
            success = final_state == STATE_DONE and error_code == ERROR_OK
            if self._pending_task is pending:
                self._pending_task = None
            self._mode = MODE_NORMAL_VELOCITY

        if success:
            goal_handle.succeed()
        else:
            goal_handle.abort()

        result.success = success
        result.final_state = final_state
        result.error_code = error_code
        result.message = message
        return result

    def _execute_lift_control(self, goal_handle) -> LiftControl.Result:
        goal = goal_handle.request
        result = LiftControl.Result()

        target_h_mm = int(goal.target_h_mm)
        mask = int(goal.mask)

        # 第一版只允许 bit0/bit1/bit2，且至少选择一个升降机构。
        if mask == 0 or (mask & ~0x07):
            goal_handle.abort()
            result.success = False
            result.final_state = STATE_ERROR
            result.error_code = ERROR_MOTOR_ERROR
            result.message = f"invalid lift mask=0x{mask:02X}, valid: 0x01/0x02/0x04/0x07"
            return result

        timeout_sec = float(goal.timeout_sec) if goal.timeout_sec and goal.timeout_sec > 0 else float(
            self.get_parameter("task_default_timeout_sec").value
        )

        seq = self._next_task_seq()
        frame = build_lift_control(seq=seq, target_h_mm=target_h_mm, mask=mask)

        pending = PendingTask(
            kind=TASK_KIND_LIFT,
            seq=seq,
            frame=frame,
            task_id=mask,
            cmd_text=f"LIFT target_h={target_h_mm} mask=0x{mask:02X}",
            timeout_sec=timeout_sec,
            goal_handle=goal_handle,
        )

        with self._state_lock:
            if self._pending_task is not None:
                goal_handle.abort()
                result.success = False
                result.final_state = STATE_ERROR
                result.error_code = ERROR_MOTOR_ERROR
                result.message = "another reliable task is running"
                return result
            self._mode = MODE_TASK
            self._pending_task = pending

        self.get_logger().info(
            f"Start LIFT_CONTROL seq={seq} target_h={target_h_mm} "
            f"mask=0x{mask:02X} timeout={timeout_sec:.2f}s"
        )
        self._send_zero_velocity()
        pending.last_send_time = self._now_monotonic()
        self._write_frame(frame, "lift_control")

        pending.done_event.wait(timeout=timeout_sec + 1.0)

        with self._state_lock:
            final_state = pending.final_state
            error_code = pending.error_code
            message = pending.message or "lift_control ended without final message"
            success = final_state == STATE_DONE and error_code == ERROR_OK
            if self._pending_task is pending:
                self._pending_task = None
            self._mode = MODE_NORMAL_VELOCITY

        if success:
            goal_handle.succeed()
        else:
            goal_handle.abort()

        result.success = success
        result.final_state = final_state
        result.error_code = error_code
        result.message = message
        return result

    def _publish_step_feedback(self, pending: PendingTask, state: int, message: str) -> None:
        feedback = StepCommand.Feedback()
        feedback.state = state
        feedback.message = message
        try:
            pending.goal_handle.publish_feedback(feedback)
        except Exception as exc:
            self.get_logger().warn(f"Failed to publish step action feedback: {exc}")

    def _publish_lift_feedback(self, pending: PendingTask, status, message: str) -> None:
        feedback = LiftControl.Feedback()
        feedback.state = int(status.state)
        feedback.lift1_mm = int(status.lift1_mm)
        feedback.lift2_mm = int(status.lift2_mm)
        feedback.lift3_mm = int(status.lift3_mm)
        feedback.message = message
        try:
            pending.goal_handle.publish_feedback(feedback)
        except Exception as exc:
            self.get_logger().warn(f"Failed to publish lift action feedback: {exc}")

    def _publish_lift_status_msg(self, status, is_task_feedback: bool, message: str) -> None:
        msg = LiftStatusMsg()
        msg.stamp = self.get_clock().now().to_msg()
        msg.lift1_mm = int(status.lift1_mm)
        msg.lift2_mm = int(status.lift2_mm)
        msg.lift3_mm = int(status.lift3_mm)
        msg.avg_height_mm = int(status.avg_height_mm)
        msg.max_diff_mm = int(status.max_diff_mm)
        msg.state = int(status.state)
        msg.error_code = int(status.error_code)
        msg.is_task_feedback = bool(is_task_feedback)
        msg.seq = int(status.seq)
        msg.message = message
        self._lift_status_pub.publish(msg)

    def _reliable_timer_cb(self) -> None:
        with self._state_lock:
            pending = self._pending_task
            if pending is None or pending.done_event.is_set():
                return
            now = self._now_monotonic()
            ack_timeout_sec = float(self.get_parameter("task_ack_timeout_sec").value)
            max_retries = int(self.get_parameter("task_max_retries").value)

            if now - pending.created_time > pending.timeout_sec:
                pending.final_state = STATE_ERROR
                pending.error_code = ERROR_MOTOR_ERROR
                pending.message = f"{pending.kind} task timeout after {pending.timeout_sec:.2f}s"
                pending.done_event.set()
                self.get_logger().error(pending.message)
                return

            if not pending.ack_received and now - pending.last_send_time > ack_timeout_sec:
                if pending.retry_count < max_retries:
                    pending.retry_count += 1
                    pending.last_send_time = now
                    frame = pending.frame
                    seq = pending.seq
                    kind = pending.kind
                else:
                    pending.final_state = STATE_ERROR
                    pending.error_code = ERROR_MOTOR_ERROR
                    pending.message = f"ACK timeout, retries exceeded seq={pending.seq}"
                    pending.done_event.set()
                    self.get_logger().error(pending.message)
                    return
            else:
                return

        self.get_logger().warn(f"ACK timeout, resend {kind} task seq={seq}, retry={pending.retry_count}")
        self._write_frame(frame, f"{kind}_resend")

    def _serial_read_loop(self) -> None:
        while not self._stop_event.is_set():
            try:
                chunk = self._serial.read(FRAME_LEN)
                if not chunk:
                    continue
                self._rx_buffer.extend(chunk)
                frames = extract_frames(self._rx_buffer)
                for frame in frames:
                    self._handle_rx_frame(frame)
            except Exception as exc:
                if not self._stop_event.is_set():
                    self.get_logger().error(f"Serial read error: {exc}")
                    time.sleep(0.05)

    def _handle_rx_frame(self, frame) -> None:
        if bool(self.get_parameter("log_rx_frames").value):
            self.get_logger().info(f"RX: {frame_to_hex(frame.raw)}")

        if frame.cmd_type == CMD_STEP_STATUS:
            self._handle_step_status(frame)
            return

        if frame.cmd_type == CMD_LIFT_STATUS:
            self._handle_lift_status(frame)
            return

        self.get_logger().debug(f"Ignore RX cmd_type=0x{frame.cmd_type:02X}")

    def _handle_step_status(self, frame) -> None:
        try:
            status = parse_step_status(frame)
        except Exception as exc:
            self.get_logger().warn(f"Invalid STEP_STATUS frame: {exc}")
            return

        with self._state_lock:
            pending = self._pending_task
            if pending is None:
                self.get_logger().warn(f"STEP_STATUS seq={status.seq} but no pending task")
                return
            if pending.kind != TASK_KIND_STEP:
                self.get_logger().warn(
                    f"Ignore STEP_STATUS seq={status.seq}; pending task kind={pending.kind}"
                )
                return
            if status.seq != pending.seq:
                self.get_logger().warn(f"Ignore STEP_STATUS seq={status.seq}, pending seq={pending.seq}")
                return
            message = (
                f"STEP_STATUS seq={status.seq} cmd={status.step_cmd_name} "
                f"state={status.state_name} error=0x{status.error_code:02X} progress={status.progress}"
            )

            if status.state in (STATE_ACK, STATE_RUNNING):
                pending.ack_received = True
                self._publish_step_feedback(pending, status.state, message)
                self.get_logger().info(message)
                return

            if status.state == STATE_DONE:
                pending.ack_received = True
                pending.final_state = STATE_DONE
                pending.error_code = status.error_code
                pending.message = message
                pending.done_event.set()
                self._publish_step_feedback(pending, status.state, message)
                self.get_logger().info(message)
                return

            if status.state == STATE_ERROR:
                pending.ack_received = True
                pending.final_state = STATE_ERROR
                pending.error_code = status.error_code or ERROR_MOTOR_ERROR
                pending.message = message
                pending.done_event.set()
                self._publish_step_feedback(pending, status.state, message)
                self.get_logger().error(message)
                return

            self.get_logger().warn(f"Unknown step state: {message}")

    def _handle_lift_status(self, frame) -> None:
        try:
            status = parse_lift_status(frame)
        except Exception as exc:
            self.get_logger().warn(f"Invalid LIFT_STATUS frame: {exc}")
            return

        message = (
            f"LIFT_STATUS seq={status.seq} "
            f"lift=({status.lift1_mm},{status.lift2_mm},{status.lift3_mm}) "
            f"avg={status.avg_height_mm} diff={status.max_diff_mm} "
            f"state={status.state_name} error=0x{status.error_code:02X}"
        )

        # 所有 LIFT_STATUS 都发布到 topic：
        #   seq=0    : STM32 主动周期状态上报
        #   seq!=0   : LIFT_CONTROL 任务反馈
        is_task_feedback = status.seq != 0
        self._publish_lift_status_msg(status, is_task_feedback, message)

        with self._state_lock:
            pending = self._pending_task
            if pending is None:
                if status.seq != 0:
                    self.get_logger().warn(f"LIFT_STATUS seq={status.seq} but no pending task")
                return

            if pending.kind != TASK_KIND_LIFT:
                # 正在执行 STEP_COMMAND 时，LIFT_STATUS 仍然会发布成 topic，
                # 但不参与当前 step action 的状态机。
                return

            if status.seq != pending.seq:
                if status.seq != 0:
                    self.get_logger().warn(f"Ignore LIFT_STATUS seq={status.seq}, pending seq={pending.seq}")
                return

            if status.state in (STATE_ACK, STATE_RUNNING):
                pending.ack_received = True
                self._publish_lift_feedback(pending, status, message)
                self.get_logger().info(message)
                return

            if status.state == STATE_DONE:
                pending.ack_received = True
                pending.final_state = STATE_DONE
                pending.error_code = status.error_code
                pending.message = message
                pending.done_event.set()
                self._publish_lift_feedback(pending, status, message)
                self.get_logger().info(message)
                return

            if status.state == STATE_ERROR:
                pending.ack_received = True
                pending.final_state = STATE_ERROR
                pending.error_code = status.error_code or ERROR_MOTOR_ERROR
                pending.message = message
                pending.done_event.set()
                self._publish_lift_feedback(pending, status, message)
                self.get_logger().error(message)
                return

            self.get_logger().warn(f"Unknown lift state: {message}")

    def _on_estop(self, request: EStop.Request, response: EStop.Response) -> EStop.Response:
        if not request.trigger:
            response.success = True
            response.message = "ESTOP trigger=false, no frame sent"
            return response

        repeat = int(self.get_parameter("estop_repeat_count").value)
        seq = self._next_task_seq()
        frame = build_estop(seq)
        for _ in range(max(1, repeat)):
            self._write_frame(frame, "estop")
            time.sleep(0.01)

        with self._state_lock:
            pending = self._pending_task
            self._mode = MODE_TASK
            if pending is not None and not pending.done_event.is_set():
                pending.final_state = STATE_ERROR
                pending.error_code = ERROR_MOTOR_ERROR
                pending.message = "ESTOP sent by host"
                pending.done_event.set()

        response.success = True
        response.message = f"ESTOP sent seq={seq}, repeat={repeat}"
        self.get_logger().error(response.message)
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ChassisSerialNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()