from __future__ import annotations

import threading
import time
from typing import Optional

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

try:
    import serial
except ImportError:
    serial = None

from .protocol import (
    CMD_CHASSIS_VEL,
    CMD_ESTOP,
    CMD_LIFT_CONTROL,
    CMD_STEP_COMMAND,
    ERROR_MOTOR_ERROR,
    ERROR_OK,
    FRAME_LEN,
    STATE_ACK,
    STATE_DONE,
    STATE_ERROR,
    STATE_NAME,
    STATE_RUNNING,
    STEP_CMD_ID_TO_NAME,
    build_lift_status,
    build_step_status,
    decode_chassis_vel_data,
    decode_lift_control_data,
    decode_step_command_data,
    extract_frames,
    frame_to_hex,
)


class MockChassisSTM32(Node):
    """Mock STM32 for local socat-based integration tests.

    It supports:
      - CHASSIS_VEL_CMD: decode and log only, no ACK.
      - STEP_COMMAND: ACK -> RUNNING -> DONE/ERROR.
      - LIFT_CONTROL v1: ACK -> RUNNING -> DONE/ERROR.
      - LIFT_STATUS seq=0 periodic reports.
      - ESTOP: set mock task states to ERROR.
    """

    def __init__(self) -> None:
        super().__init__("mock_chassis_stm32")

        self.declare_parameter("port", "/tmp/r2_chassis_stm32")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("serial_timeout_sec", 0.02)
        self.declare_parameter("simulate_error", False)
        self.declare_parameter("drop_first_ack", False)
        self.declare_parameter("running_delay_sec", 0.3)
        self.declare_parameter("done_delay_sec", 1.0)
        self.declare_parameter("lift_status_rate_hz", 5.0)
        self.declare_parameter("lift_motion_delay_sec", 1.0)
        self.declare_parameter("log_velocity", True)
        self.declare_parameter("log_rx_frames", False)
        self.declare_parameter("log_tx_frames", False)
        self.declare_parameter("log_periodic_lift_status", False)

        if serial is None:
            raise RuntimeError("pyserial is not installed. Install with: sudo apt install python3-serial")

        self.port = self.get_parameter("port").value
        self.baudrate = int(self.get_parameter("baudrate").value)
        timeout = float(self.get_parameter("serial_timeout_sec").value)
        self._serial = serial.Serial(self.port, self.baudrate, timeout=timeout)
        self.get_logger().info(f"Mock STM32 opened serial port {self.port} @ {self.baudrate}")

        self._rx_buffer = bytearray()
        self._stop_event = threading.Event()
        self._write_lock = threading.Lock()
        self._state_lock = threading.RLock()

        # STEP_COMMAND mock state.
        self._current_seq: Optional[int] = None
        self._current_command_data: Optional[bytes] = None
        self._current_step_cmd: int = 0
        self._current_state: int = STATE_DONE
        self._current_error: int = ERROR_OK
        self._current_progress: int = 0

        # LIFT_CONTROL mock state.
        self._current_lift_seq: Optional[int] = None
        self._current_lift_data: Optional[bytes] = None
        self._lift1_mm: int = 0
        self._lift2_mm: int = 0
        self._lift3_mm: int = 0
        self._lift_state: int = STATE_DONE
        self._lift_error: int = ERROR_OK

        # Shared "drop first ACK" test flag. It drops the first reliable-task ACK once.
        self._first_ack_dropped = False

        lift_status_rate_hz = float(self.get_parameter("lift_status_rate_hz").value)
        self._lift_status_timer = self.create_timer(
            1.0 / max(1.0, lift_status_rate_hz),
            self._periodic_lift_status_cb,
        )

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

    def _write_frame(self, frame: bytes, reason: str = "") -> None:
        with self._write_lock:
            self._serial.write(frame)
            self._serial.flush()
        if bool(self.get_parameter("log_tx_frames").value):
            self.get_logger().info(f"TX {reason}: {frame_to_hex(frame)}")

    def _reliable_task_busy(self) -> bool:
        return self._current_state in (STATE_ACK, STATE_RUNNING) or self._lift_state in (STATE_ACK, STATE_RUNNING)

    def _send_step_status(
        self,
        seq: int,
        step_cmd: int,
        state: int,
        error_code: int = ERROR_OK,
        progress: int = 0,
    ) -> None:
        frame = build_step_status(seq, step_cmd, state, error_code, progress)
        self._write_frame(frame, f"STEP_STATUS/{STATE_NAME.get(state, state)}")
        self.get_logger().info(
            f"Mock TX STEP_STATUS seq={seq} cmd={STEP_CMD_ID_TO_NAME.get(step_cmd, step_cmd)} "
            f"state={STATE_NAME.get(state, state)} error=0x{error_code:02X} progress={progress}"
        )

    def _send_lift_status(self, seq: int, state: int, error_code: int = ERROR_OK) -> None:
        with self._state_lock:
            lift1 = self._lift1_mm
            lift2 = self._lift2_mm
            lift3 = self._lift3_mm

        frame = build_lift_status(
            seq=seq,
            lift1_mm=lift1,
            lift2_mm=lift2,
            lift3_mm=lift3,
            state=state,
            error_code=error_code,
        )
        self._write_frame(frame, f"LIFT_STATUS/{STATE_NAME.get(state, state)}")
        self.get_logger().info(
            f"Mock TX LIFT_STATUS seq={seq} lift=({lift1},{lift2},{lift3}) "
            f"state={STATE_NAME.get(state, state)} error=0x{error_code:02X}"
        )

    def _periodic_lift_status_cb(self) -> None:
        with self._state_lock:
            frame = build_lift_status(
                seq=0,
                lift1_mm=self._lift1_mm,
                lift2_mm=self._lift2_mm,
                lift3_mm=self._lift3_mm,
                state=self._lift_state,
                error_code=self._lift_error,
            )
            state = self._lift_state
            error = self._lift_error
            lift1 = self._lift1_mm
            lift2 = self._lift2_mm
            lift3 = self._lift3_mm

        self._write_frame(frame, "LIFT_STATUS/periodic")
        if bool(self.get_parameter("log_periodic_lift_status").value):
            self.get_logger().info(
                f"Mock TX periodic LIFT_STATUS seq=0 lift=({lift1},{lift2},{lift3}) "
                f"state={STATE_NAME.get(state, state)} error=0x{error:02X}"
            )

    def _serial_read_loop(self) -> None:
        while not self._stop_event.is_set():
            try:
                chunk = self._serial.read(FRAME_LEN)
                if not chunk:
                    continue
                self._rx_buffer.extend(chunk)
                for frame in extract_frames(self._rx_buffer):
                    self._handle_frame(frame)
            except Exception as exc:
                if not self._stop_event.is_set():
                    self.get_logger().error(f"Mock serial read error: {exc}")
                    time.sleep(0.05)

    def _handle_frame(self, frame) -> None:
        if bool(self.get_parameter("log_rx_frames").value):
            self.get_logger().info(f"RX: {frame_to_hex(frame.raw)}")

        if frame.cmd_type == CMD_CHASSIS_VEL:
            vx, vy, omega, mode = decode_chassis_vel_data(frame.data)
            if bool(self.get_parameter("log_velocity").value):
                self.get_logger().info(
                    f"Mock RX CHASSIS_VEL seq={frame.seq} vx={vx} vy={vy} omega={omega} mode={mode}"
                )
            return

        if frame.cmd_type == CMD_LIFT_CONTROL:
            self._handle_lift_control(frame.seq, frame.data)
            return

        if frame.cmd_type == CMD_STEP_COMMAND:
            self._handle_step_command(frame.seq, frame.data)
            return

        if frame.cmd_type == CMD_ESTOP:
            self.get_logger().error(f"Mock RX ESTOP seq={frame.seq}; stop chassis/lift/climb immediately")
            with self._state_lock:
                self._current_state = STATE_ERROR
                self._current_error = ERROR_MOTOR_ERROR
                self._current_progress = 0
                self._lift_state = STATE_ERROR
                self._lift_error = ERROR_MOTOR_ERROR
            return

        self.get_logger().warn(f"Mock ignore unknown cmd_type=0x{frame.cmd_type:02X}, seq={frame.seq}")

    def _handle_step_command(self, seq: int, data: bytes) -> None:
        step_cmd, delta_h, edge_id, flags = decode_step_command_data(data)
        cmd_name = STEP_CMD_ID_TO_NAME.get(step_cmd, f"UNKNOWN({step_cmd})")

        with self._state_lock:
            duplicate = self._current_seq == seq and self._current_command_data == data
            if duplicate:
                state = self._current_state
                error = self._current_error
                progress = self._current_progress
                self.get_logger().warn(
                    f"Mock duplicate STEP_COMMAND seq={seq}; resend current state={STATE_NAME.get(state, state)}"
                )
                self._send_step_status(seq, step_cmd, state, error, progress)
                return

            if self._reliable_task_busy():
                self.get_logger().warn(
                    f"Mock busy, reject new STEP_COMMAND seq={seq}; first version returns MOTOR_ERROR"
                )
                self._send_step_status(seq, step_cmd, STATE_ERROR, ERROR_MOTOR_ERROR, 0)
                return

            self._current_seq = seq
            self._current_command_data = data
            self._current_step_cmd = step_cmd
            self._current_state = STATE_ACK
            self._current_error = ERROR_OK
            self._current_progress = 0

        self.get_logger().info(
            f"Mock RX STEP_COMMAND seq={seq} cmd={cmd_name} delta_h={delta_h} edge_id={edge_id} flags=0x{flags:02X}"
        )

        if bool(self.get_parameter("drop_first_ack").value) and not self._first_ack_dropped:
            self._first_ack_dropped = True
            self.get_logger().warn("Mock drop first reliable-task ACK once, for host resend test")
        else:
            self._send_step_status(seq, step_cmd, STATE_ACK, ERROR_OK, 0)

        threading.Thread(target=self._finish_step_task, args=(seq, step_cmd), daemon=True).start()

    def _finish_step_task(self, seq: int, step_cmd: int) -> None:
        running_delay = float(self.get_parameter("running_delay_sec").value)
        done_delay = float(self.get_parameter("done_delay_sec").value)
        simulate_error = bool(self.get_parameter("simulate_error").value)

        time.sleep(max(0.0, running_delay))
        with self._state_lock:
            if self._current_seq != seq:
                return
            self._current_state = STATE_RUNNING
            self._current_progress = 0
        self._send_step_status(seq, step_cmd, STATE_RUNNING, ERROR_OK, 0)

        time.sleep(max(0.0, done_delay))
        with self._state_lock:
            if self._current_seq != seq:
                return
            if simulate_error:
                self._current_state = STATE_ERROR
                self._current_error = ERROR_MOTOR_ERROR
                self._current_progress = 0
                state = STATE_ERROR
                error = ERROR_MOTOR_ERROR
                progress = 0
            else:
                self._current_state = STATE_DONE
                self._current_error = ERROR_OK
                self._current_progress = 0
                state = STATE_DONE
                error = ERROR_OK
                progress = 0
        self._send_step_status(seq, step_cmd, state, error, progress)

    def _handle_lift_control(self, seq: int, data: bytes) -> None:
        target_h, mask = decode_lift_control_data(data)

        with self._state_lock:
            duplicate = self._current_lift_seq == seq and self._current_lift_data == data
            if duplicate:
                state = self._lift_state
                error = self._lift_error
                self.get_logger().warn(
                    f"Mock duplicate LIFT_CONTROL seq={seq}; resend current state={STATE_NAME.get(state, state)}"
                )
                self._send_lift_status(seq, state, error)
                return

            if mask == 0 or (mask & ~0x07):
                self.get_logger().warn(f"Mock reject LIFT_CONTROL seq={seq}; invalid mask=0x{mask:02X}")
                self._send_lift_status(seq, STATE_ERROR, ERROR_MOTOR_ERROR)
                return

            if self._reliable_task_busy():
                self.get_logger().warn(
                    f"Mock busy, reject new LIFT_CONTROL seq={seq}; first version returns MOTOR_ERROR"
                )
                self._send_lift_status(seq, STATE_ERROR, ERROR_MOTOR_ERROR)
                return

            self._current_lift_seq = seq
            self._current_lift_data = data
            self._lift_state = STATE_ACK
            self._lift_error = ERROR_OK

        self.get_logger().info(f"Mock RX LIFT_CONTROL seq={seq} target_h={target_h} mask=0x{mask:02X}")

        if bool(self.get_parameter("drop_first_ack").value) and not self._first_ack_dropped:
            self._first_ack_dropped = True
            self.get_logger().warn("Mock drop first reliable-task ACK once, for host resend test")
        else:
            self._send_lift_status(seq, STATE_ACK, ERROR_OK)

        threading.Thread(target=self._finish_lift_task, args=(seq, target_h, mask), daemon=True).start()

    def _finish_lift_task(self, seq: int, target_h: int, mask: int) -> None:
        running_delay = float(self.get_parameter("running_delay_sec").value)
        lift_motion_delay = float(self.get_parameter("lift_motion_delay_sec").value)
        simulate_error = bool(self.get_parameter("simulate_error").value)

        time.sleep(max(0.0, running_delay))
        with self._state_lock:
            if self._current_lift_seq != seq:
                return
            self._lift_state = STATE_RUNNING
        self._send_lift_status(seq, STATE_RUNNING, ERROR_OK)

        time.sleep(max(0.0, lift_motion_delay))
        with self._state_lock:
            if self._current_lift_seq != seq:
                return

            if simulate_error:
                self._lift_state = STATE_ERROR
                self._lift_error = ERROR_MOTOR_ERROR
                state = STATE_ERROR
                error = ERROR_MOTOR_ERROR
            else:
                if mask & 0x01:
                    self._lift1_mm = target_h
                if mask & 0x02:
                    self._lift2_mm = target_h
                if mask & 0x04:
                    self._lift3_mm = target_h

                self._lift_state = STATE_DONE
                self._lift_error = ERROR_OK
                state = STATE_DONE
                error = ERROR_OK

        self._send_lift_status(seq, state, error)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MockChassisSTM32()
    executor = MultiThreadedExecutor(num_threads=3)
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