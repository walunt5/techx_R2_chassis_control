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
    CMD_STEP_COMMAND,
    ERROR_MOTOR_ERROR,
    ERROR_OK,
    FRAME_LEN,
    STATE_ACK,
    STATE_DONE,
    STATE_ERROR,
    STATE_NAME,
    STATE_RUNNING,
    build_step_status,
    decode_chassis_vel_data,
    decode_step_command_data,
    extract_frames,
    frame_to_hex,
    STEP_CMD_ID_TO_NAME,
)


class MockChassisSTM32(Node):
    def __init__(self) -> None:
        super().__init__("mock_chassis_stm32")

        self.declare_parameter("port", "/tmp/r2_chassis_stm32")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("serial_timeout_sec", 0.02)
        self.declare_parameter("simulate_error", False)
        self.declare_parameter("drop_first_ack", False)
        self.declare_parameter("running_delay_sec", 0.3)
        self.declare_parameter("done_delay_sec", 1.0)
        self.declare_parameter("log_velocity", True)
        self.declare_parameter("log_rx_frames", False)
        self.declare_parameter("log_tx_frames", False)

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

        self._current_seq: Optional[int] = None
        self._current_command_data: Optional[bytes] = None
        self._current_step_cmd: int = 0
        self._current_state: int = STATE_DONE
        self._current_error: int = ERROR_OK
        self._current_progress: int = 0
        self._first_ack_dropped = False

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

    def _send_step_status(self, seq: int, step_cmd: int, state: int, error_code: int = ERROR_OK, progress: int = 0) -> None:
        frame = build_step_status(seq, step_cmd, state, error_code, progress)
        self._write_frame(frame, f"STEP_STATUS/{STATE_NAME.get(state, state)}")
        self.get_logger().info(
            f"Mock TX STEP_STATUS seq={seq} cmd={STEP_CMD_ID_TO_NAME.get(step_cmd, step_cmd)} "
            f"state={STATE_NAME.get(state, state)} error=0x{error_code:02X} progress={progress}"
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
                self.get_logger().info(f"Mock RX CHASSIS_VEL seq={frame.seq} vx={vx} vy={vy} omega={omega} mode={mode}")
            return

        if frame.cmd_type == CMD_STEP_COMMAND:
            self._handle_step_command(frame.seq, frame.data)
            return

        if frame.cmd_type == CMD_ESTOP:
            self.get_logger().error(f"Mock RX ESTOP seq={frame.seq}; stop chassis/lift/climb immediately")
            with self._state_lock:
                self._current_state = STATE_ERROR
                self._current_error = ERROR_MOTOR_ERROR
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

            if self._current_state in (STATE_ACK, STATE_RUNNING):
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
            self.get_logger().warn("Mock drop first ACK once, for host resend test")
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
            self._current_progress = 50
        self._send_step_status(seq, step_cmd, STATE_RUNNING, ERROR_OK, 50)

        time.sleep(max(0.0, done_delay))
        with self._state_lock:
            if self._current_seq != seq:
                return
            if simulate_error:
                self._current_state = STATE_ERROR
                self._current_error = ERROR_MOTOR_ERROR
                self._current_progress = 50
                state = STATE_ERROR
                error = ERROR_MOTOR_ERROR
                progress = 50
            else:
                self._current_state = STATE_DONE
                self._current_error = ERROR_OK
                self._current_progress = 100
                state = STATE_DONE
                error = ERROR_OK
                progress = 100
        self._send_step_status(seq, step_cmd, state, error, progress)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MockChassisSTM32()
    executor = MultiThreadedExecutor(num_threads=2)
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
