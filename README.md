# techx_R2_chassis_control

R2 底盘 / 升降 / 爬台阶 STM32 串口通信 ROS 2 Humble 工程。

第一版目标：跑通最小闭环，不实现完整复杂协议。

- `/cmd_vel` -> `CHASSIS_VEL_CMD` 周期速度流
- `/r2_chassis/step_command` -> `STEP_COMMAND` 可靠任务
- `/r2_chassis/estop` -> `ESTOP` 急停服务
- `mock_chassis_stm32` 支持 socat 虚拟串口测试

## Packages

- `techx_r2_chassis_interfaces`: Action / Service 接口
- `techx_r2_chassis_control`: Python 串口节点、协议、mock、测试客户端

## Quick start with virtual serial ports

```bash
sudo apt update
sudo apt install -y python3-serial socat

mkdir -p ~/r2_chassis_control_ws/src
cd ~/r2_chassis_control_ws/src
# copy this repository here
cd ~/r2_chassis_control_ws
colcon build --packages-select techx_r2_chassis_interfaces techx_r2_chassis_control --symlink-install
source install/setup.bash
```

Terminal 1:

```bash
socat -d -d pty,raw,echo=0,link=/tmp/r2_chassis_host pty,raw,echo=0,link=/tmp/r2_chassis_stm32
```

Terminal 2:

```bash
ros2 launch techx_r2_chassis_control mock_chassis.launch.py
```

Terminal 3:

```bash
ros2 run techx_r2_chassis_control test_cmd_vel_pub --vx 0.1 --duration-sec 3.0
ros2 run techx_r2_chassis_control test_step_command_client CLIMB_200 --current-h 0 --target-h 200 --edge-id 1
ros2 service call /r2_chassis/estop techx_r2_chassis_interfaces/srv/EStop "{trigger: true}"
```
