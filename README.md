# techx_R2_chassis_control

R2 底盘 / 升降 / 爬台阶 STM32 串口通信 ROS 2 Humble 工程。

本工程用于封装 R2 机器人的底盘 STM32 串口通信。上层导航、行为树、Web 控制不直接操作串口，而是通过 ROS 2 topic / action / service 调用本工程；本工程内部负责打包固定 18 字节串口帧、发送速度流、执行可靠任务、解析 STM32 反馈。

第一版目标是先跑通最小闭环，不实现完整复杂协议。

## 第一版支持功能

* `/cmd_vel` → `CHASSIS_VEL_CMD` 周期速度流
* `/r2_chassis/step_command` → `STEP_COMMAND` 可靠任务
* `/r2_chassis/estop` → `ESTOP` 急停服务
* `mock_chassis_stm32` 支持虚拟串口测试
* `mock_chassis.launch.py` 可自动启动 `socat`、mock STM32 和上位机串口节点

## Packages

本工程包含两个 ROS 2 package：

```text
techx_r2_chassis_interfaces
techx_r2_chassis_control
```

### `techx_r2_chassis_interfaces`

用于定义 ROS 2 Action / Service 接口。

包含：

```text
action/StepCommand.action
srv/EStop.srv
```

### `techx_r2_chassis_control`

用于实现底盘串口通信逻辑。

包含：

```text
protocol.py
chassis_serial_node.py
mock_chassis_stm32.py
test_step_command_client.py
test_cmd_vel_pub.py
test_estop_client.py
protocol_demo.py
launch/
config/
```

## ROS 2 接口

### 1. `/cmd_vel`

```text
topic: /cmd_vel
type: geometry_msgs/msg/Twist
```

用于普通底盘速度控制。

`chassis_serial_node` 会订阅 `/cmd_vel`，并以固定频率向 STM32 发送 `CHASSIS_VEL_CMD` 速度帧。

默认逻辑：

```text
20~50Hz 周期发送速度帧
超过 200ms 没有新的 /cmd_vel 自动发送 0 速度
执行可靠任务时进入 TaskMode，暂停普通速度流或只发送 0 速度
```

### 2. `/r2_chassis/step_command`

```text
action: /r2_chassis/step_command
type: techx_r2_chassis_interfaces/action/StepCommand
```

用于行为树下发底盘动作级命令。

第一版支持：

```text
MOVE_FLAT
CLIMB_200
CLIMB_400
DESCEND_200
DESCEND_400
```

行为树后续只需要调用这个 Action，不需要直接操作串口。

### 3. `/r2_chassis/estop`

```text
service: /r2_chassis/estop
type: techx_r2_chassis_interfaces/srv/EStop
```

用于急停。

调用后，上位机会立即向 STM32 发送 `ESTOP` 串口帧。

## 串口协议简化说明

第一版使用固定 18 字节帧：

```text
AA 55 frame_type seq cmd_type data[8] reserved[3] crc16
```

字段含义：

```text
Byte0      0xAA
Byte1      0x55
Byte2      frame_type
Byte3      seq
Byte4      cmd_type
Byte5~12   data[8]
Byte13~15  reserved[3]
Byte16~17  CRC16_MODBUS，小端
```

第一版底盘 cmd_type：

```text
0x30 = CHASSIS_VEL_CMD
0x32 = STEP_COMMAND
0x42 = STEP_STATUS
0xE0 = ESTOP
```

### `CHASSIS_VEL_CMD`

上位机周期发送，不等待 ACK。

```text
data0~1: vx_mm_s，int16，小端
data2~3: vy_mm_s，int16，小端
data4~5: omega_mrad_s，int16，小端
data6: mode，第一版填 0
data7: reserved，填 0
```

### `STEP_COMMAND`

上位机发送可靠任务，需要 STM32 返回 ACK / RUNNING / DONE / ERROR。

```text
data0: step_cmd
data1~2: delta_h_mm，int16，小端
data3: edge_id
data4: flags
data5~7: reserved，填 0
```

第一版 `step_cmd`：

```text
0 = MOVE_FLAT
1 = CLIMB_200
2 = CLIMB_400
3 = DESCEND_200
4 = DESCEND_400
```

### `STEP_STATUS`

STM32 返回台阶动作状态。

```text
data0: step_cmd
data1: state
data2: error_code
data3: progress
data4~7: reserved，填 0
```

第一版 `state`：

```text
1 = ACK
2 = RUNNING
3 = DONE
4 = ERROR
```

第一版 `error_code`：

```text
0x00 = OK
0x08 = MOTOR_ERROR
```

## 安装依赖

```bash
sudo apt update
sudo apt install -y python3-serial socat
```

## 编译工程

```bash
mkdir -p ~/r2_chassis_control_ws/src
cd ~/r2_chassis_control_ws/src

# 将本工程复制到 src 目录下
# 目录结构应类似：
# ~/r2_chassis_control_ws/src/techx_r2_chassis_control/
# ~/r2_chassis_control_ws/src/techx_r2_chassis_interfaces/

cd ~/r2_chassis_control_ws

colcon build --packages-select \
  techx_r2_chassis_interfaces \
  techx_r2_chassis_control \
  --symlink-install

source install/setup.bash
```

检查接口是否生成成功：

```bash
ros2 interface show techx_r2_chassis_interfaces/action/StepCommand
ros2 interface show techx_r2_chassis_interfaces/srv/EStop
```

检查节点是否安装成功：

```bash
ros2 pkg executables techx_r2_chassis_control
```

正常应看到类似：

```text
techx_r2_chassis_control chassis_serial_node
techx_r2_chassis_control mock_chassis_stm32
techx_r2_chassis_control test_step_command_client
techx_r2_chassis_control test_cmd_vel_pub
techx_r2_chassis_control test_estop_client
techx_r2_chassis_control protocol_demo
```

## 使用方式一：虚拟串口 mock 测试

`mock_chassis.launch.py` 用于电脑本地测试。

它会自动启动：

```text
socat 虚拟串口
mock_chassis_stm32
chassis_serial_node
```

默认虚拟串口：

```text
/tmp/r2_chassis_host
/tmp/r2_chassis_stm32
```

启动 mock 测试：

```bash
cd ~/r2_chassis_control_ws
source install/setup.bash

ros2 launch techx_r2_chassis_control mock_chassis.launch.py
```

如果启动成功，会自动创建虚拟串口，并启动上位机节点和 mock STM32 节点。

## 测试 `/cmd_vel`

另开一个终端：

```bash
cd ~/r2_chassis_control_ws
source install/setup.bash

ros2 run techx_r2_chassis_control test_cmd_vel_pub \
  --vx 0.1 \
  --duration-sec 3.0
```

含义：

```text
发布 vx = 0.1 m/s
上位机转换为 vx = 100 mm/s
通过 CHASSIS_VEL_CMD 周期发送给 mock STM32
```

mock 终端应能看到速度帧打印。

## 测试 `CLIMB_200`

```bash
cd ~/r2_chassis_control_ws
source install/setup.bash

ros2 run techx_r2_chassis_control test_step_command_client CLIMB_200 \
  --from-block A \
  --target-block B \
  --current-h 0 \
  --target-h 200 \
  --edge-id 1 \
  --timeout-sec 10.0
```

正常流程：

```text
Action goal 发送
chassis_serial_node 进入 TaskMode
发送 0 速度
发送 STEP_COMMAND
mock 返回 ACK
mock 返回 RUNNING
mock 返回 DONE
Action 返回 success=true
chassis_serial_node 回到 NormalVelocity
```

## 测试其他台阶动作

### MOVE_FLAT

```bash
ros2 run techx_r2_chassis_control test_step_command_client MOVE_FLAT \
  --current-h 0 \
  --target-h 0 \
  --edge-id 1
```

### CLIMB_400

```bash
ros2 run techx_r2_chassis_control test_step_command_client CLIMB_400 \
  --current-h 0 \
  --target-h 400 \
  --edge-id 1
```

### DESCEND_200

```bash
ros2 run techx_r2_chassis_control test_step_command_client DESCEND_200 \
  --current-h 200 \
  --target-h 0 \
  --edge-id 1
```

### DESCEND_400

```bash
ros2 run techx_r2_chassis_control test_step_command_client DESCEND_400 \
  --current-h 400 \
  --target-h 0 \
  --edge-id 1
```

## 测试 ERROR

启动 mock 时打开错误模拟：

```bash
ros2 launch techx_r2_chassis_control mock_chassis.launch.py simulate_error:=true
```

然后发送动作：

```bash
ros2 run techx_r2_chassis_control test_step_command_client CLIMB_200 \
  --current-h 0 \
  --target-h 200 \
  --edge-id 1
```

预期结果：

```text
mock 返回 ACK
mock 返回 RUNNING
mock 返回 ERROR
Action 返回 success=false
error_code = 0x08
```

## 测试 ACK 重发

启动 mock 时丢弃第一次 ACK：

```bash
ros2 launch techx_r2_chassis_control mock_chassis.launch.py drop_first_ack:=true
```

然后发送动作：

```bash
ros2 run techx_r2_chassis_control test_step_command_client CLIMB_200 \
  --current-h 0 \
  --target-h 200 \
  --edge-id 1
```

预期结果：

```text
第一次 ACK 被 mock 故意丢弃
chassis_serial_node 100ms 后使用同一个 seq 重发 STEP_COMMAND
mock 收到重发帧后返回 ACK / RUNNING / DONE
Action 最终成功
```

## 测试 ESTOP 急停

```bash
ros2 service call /r2_chassis/estop \
  techx_r2_chassis_interfaces/srv/EStop \
  "{trigger: true}"
```

预期结果：

```text
chassis_serial_node 立即发送 ESTOP 帧
mock_chassis_stm32 打印 ESTOP 信息
如果当前存在 pending task，则任务失败退出
```

## 使用方式二：连接真实 STM32

连接真实 STM32 时，不使用 mock launch。

先查看真实串口：

```bash
ls /dev/ttyUSB*
ls /dev/ttyACM*
```

假设底盘 STM32 是：

```text
/dev/ttyUSB0
```

启动真实串口节点：

```bash
cd ~/r2_chassis_control_ws
source install/setup.bash

ros2 launch techx_r2_chassis_control chassis_serial.launch.py port:=/dev/ttyUSB0
```

如果是 `/dev/ttyACM0`：

```bash
ros2 launch techx_r2_chassis_control chassis_serial.launch.py port:=/dev/ttyACM0
```

## 与导航工程配合

后续如果和 `techx_R2_algorithm` 一起使用，导航工程仍然负责发布 `/cmd_vel`。

但是旧的串口通信节点不要再启动。

推荐结构：

```text
techx_R2_algorithm
  ├── 导航
  ├── 路径规划
  ├── Web 控制
  ├── 行为树
  └── 发布 /cmd_vel 或调用 /r2_chassis/step_command

techx_R2_chassis_control
  ├── 订阅 /cmd_vel
  ├── 提供 /r2_chassis/step_command
  ├── 提供 /r2_chassis/estop
  └── 独占底盘 STM32 串口
```

启动导航工程时应关闭旧串口节点，例如：

```bash
ros2 launch r2_nav_bringup r2_odin_web_nav.launch.py launch_serial:=false
```

然后单独启动新的底盘串口节点：

```bash
ros2 launch techx_r2_chassis_control chassis_serial.launch.py port:=/dev/ttyUSB0
```

不要让旧串口节点和新的 `chassis_serial_node` 同时打开同一个底盘串口。

## 常用调试命令

查看 topic：

```bash
ros2 topic list
```

查看 `/cmd_vel`：

```bash
ros2 topic echo /cmd_vel
```

查看 Action：

```bash
ros2 action list
ros2 action info /r2_chassis/step_command
```

查看 Service：

```bash
ros2 service list
ros2 service type /r2_chassis/estop
```

测试协议打包：

```bash
ros2 run techx_r2_chassis_control protocol_demo
```

## 故障排查

### 1. 找不到节点

如果执行：

```bash
ros2 pkg executables techx_r2_chassis_control
```

看不到节点，先确认是否已经 source：

```bash
source ~/r2_chassis_control_ws/install/setup.bash
```

然后重新编译：

```bash
cd ~/r2_chassis_control_ws

colcon build --packages-select \
  techx_r2_chassis_interfaces \
  techx_r2_chassis_control \
  --symlink-install

source install/setup.bash
```

### 2. 串口打不开

如果是真实 STM32，检查串口权限：

```bash
ls -l /dev/ttyUSB0
```

临时授权：

```bash
sudo chmod 666 /dev/ttyUSB0
```

长期建议把用户加入 `dialout` 组：

```bash
sudo usermod -aG dialout $USER
```

然后重新登录系统。

### 3. mock launch 启动失败，提示找不到 socat

安装 socat：

```bash
sudo apt install -y socat
```

### 4. mock launch 启动后串口仍然打不开

可能是旧的 socat 进程没有退出，或者 `/tmp/r2_chassis_host`、`/tmp/r2_chassis_stm32` 被占用。

可以先清理：

```bash
pkill socat
rm -f /tmp/r2_chassis_host /tmp/r2_chassis_stm32
```

然后重新启动：

```bash
ros2 launch techx_r2_chassis_control mock_chassis.launch.py
```

### 5. 编译出现 CMP0148 warning

如果看到类似：

```text
CMake Warning (dev) ... CMP0148 ...
```

但最后显示：

```text
Finished <<< techx_r2_chassis_interfaces
Finished <<< techx_r2_chassis_control
```

说明编译成功，可以先忽略该 warning。

## 后续扩展方向

第一版只实现最小闭环。后续可以逐步扩展：

```text
LIFT_CONTROL
LIFT_STATUS
CHASSIS_STATUS
GYM_ALIGN
完整 CLIMB_STATUS
UNSYNC
LIMIT_TRIGGERED
ACTION_TIMEOUT
ESTOP_ACTIVE
更完整 error_code
```

建议扩展顺序：

```text
1. 先稳定 /cmd_vel 速度流
2. 再稳定 STEP_COMMAND 的 ACK / RUNNING / DONE / ERROR
3. 再接入真实 STM32
4. 再接行为树 ChassisStepCommandNode
5. 最后扩展升降、对准、完整状态诊断
```

## 设计原则

```text
行为树不直接操作串口
导航只发布 /cmd_vel
ChassisStepCommandNode 只调用 /r2_chassis/step_command
chassis_serial_node 负责串口协议
STM32 负责真实底盘 / 升降 / 爬阶动作
```

第一版目标不是一次性实现完整复杂协议，而是先把：

```text
ROS 2 上位机
  ↓
串口帧
  ↓
STM32 / mock STM32
  ↓
STEP_STATUS 反馈
  ↓
ROS 2 Action 结果
```

这条闭环稳定跑通。
