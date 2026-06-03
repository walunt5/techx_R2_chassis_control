from __future__ import annotations

from .protocol import (
    build_chassis_vel_cmd,
    build_estop,
    build_step_command,
    build_step_status,
    frame_to_hex,
    parse_frame,
    parse_step_status,
)


def main() -> None:
    vel = build_chassis_vel_cmd(seq=1, vx_mm_s=100, vy_mm_s=-20, omega_mrad_s=300)
    step = build_step_command(seq=2, step_cmd=1, delta_h_mm=200, edge_id=1)
    status = build_step_status(seq=2, step_cmd=1, state=3, error_code=0, progress=100)
    estop = build_estop(seq=3)

    print("CHASSIS_VEL_CMD:", frame_to_hex(vel))
    print("STEP_COMMAND  :", frame_to_hex(step))
    print("STEP_STATUS   :", frame_to_hex(status))
    print("ESTOP         :", frame_to_hex(estop))

    parsed = parse_frame(status)
    step_status = parse_step_status(parsed)
    print("parsed STEP_STATUS:", step_status)


if __name__ == "__main__":
    main()
