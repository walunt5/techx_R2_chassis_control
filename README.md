# techx_R2_chassis_control

R2 底盘 / 升降 / 爬台阶 STM32 串口通信 ROS 2 Humble 工程。

本工程用于封装 R2 机器人的底盘 STM32 串口通信。上层导航、行为树、Web 控制不直接操作串口，而是通过 ROS 2 topic / action / service 调用本工程；本工程内部负责打包固定 18 字节串口帧、发送速度流、执行可靠任务、解析 STM32 反馈，并将 STM32 的升降状态发布为 ROS 2 topic。

第一版目标是先跑通最小闭环，不实现完整复杂协议。

## 第一版支持功能

- `/cmd_vel` → `CHASSIS_VEL_CMD` 周期速度流
- `/r2_chassis/step_command` → `STEP_COMMAND` 可靠任务
- `/r2_chassis/lift_control` → `LIFT_CONTROL` 升降高度控制可靠任务
- `/r2_chassis/lift_status` ← `LIFT_STATUS` 升降状态实时上报
- `/r2_chassis/estop` → `ESTOP` 急停服务
- `mock_chassis_stm32` 支持虚拟串口测试
- `mock_chassis.launch.py` 可自动启动 `socat`、mock STM32 和上位机串口节点

第一版升降协议采用最小设计：

```text
LIFT_CONTROL:
    只控制“目标高度 + 升降 mask”
    不包含 speed_level
    不包含复杂 lift_cmd
    不包含 STOP / UP / DOWN / SUPPORT_DOWN / RETRACT_UP
```

## Packages

本工程包含两个 ROS 2 package：

```text
techx_r2_chassis_interfaces
techx_r2_chassis_control
```

### `techx_r2_chassis_interfaces`

用于定义 ROS 2 Action / Message / Service 接口。

包含：

```text
action/StepCommand.action
action/LiftControl.action
msg/LiftStatus.msg
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
test_lift_control_client.py
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
NormalVelocity 模式：
    周期发送 /cmd_vel 对应的速度帧

/cmd_vel 超时：
    超过 cmd_vel_timeout_sec 没有新速度，自动发送 0 速度

TaskMode 模式：
    执行 STEP_COMMAND 或 LIFT_CONTROL 时，暂停普通速度流
    如果 send_zero_in_task_mode=true，则任务期间周期发送 0 速度
    如果 send_zero_in_task_mode=false，则任务期间不周期发送速度帧
```

实机第一版建议：

```text
send_zero_in_task_mode: false
```

这样进入可靠任务时，上位机只在任务开始前发一次 0 速度，然后由 STM32 自己接管升降或爬阶动作。

---

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

---

### 3. `/r2_chassis/lift_control`

```text
action: /r2_chassis/lift_control
type: techx_r2_chassis_interfaces/action/LiftControl
```

用于控制升降机构运动到指定高度。

第一版接口：

```text
# Goal
int32 target_h_mm
uint8 mask
float32 timeout_sec
---
# Result
bool success
uint8 final_state
uint16 error_code
string message
---
# Feedback
uint8 state
int32 lift1_mm
int32 lift2_mm
int32 lift3_mm
string message
```

字段说明：

```text
target_h_mm:
    目标高度，单位 mm。

mask:
    bit0 = 升降1
    bit1 = 升降2
    bit2 = 升降3

    0x01 = 只控制升降1
    0x02 = 只控制升降2
    0x04 = 只控制升降3
    0x07 = 三个升降一起控制，也就是控制整个底盘高度。

timeout_sec:
    任务超时时间。
```

示例：

```text
target_h_mm = 200
mask = 0x07

含义：
    三个升降一起运动到 200mm。
```

```text
target_h_mm = 100
mask = 0x01

含义：
    只控制升降1运动到 100mm。
```

---

### 4. `/r2_chassis/lift_status`

```text
topic: /r2_chassis/lift_status
type: techx_r2_chassis_interfaces/msg/LiftStatus
```

用于实时发布三个升降机构的状态。

消息内容：

```text
builtin_interfaces/Time stamp

int32 lift1_mm
int32 lift2_mm
int32 lift3_mm

int32 avg_height_mm
int32 max_diff_mm

uint8 state
uint16 error_code

bool is_task_feedback
uint8 seq

string message
```

字段说明：

```text
lift1_mm / lift2_mm / lift3_mm:
    三个升降机构的当前高度，单位 mm。

avg_height_mm:
    三个升降高度的平均值。

max_diff_mm:
    三个升降之间的最大高度差，用于判断是否不同步。

state:
    当前升降状态。

error_code:
    错误码。

is_task_feedback:
    true 表示这帧状态是某条 LIFT_CONTROL 的任务反馈。
    false 表示这帧状态是 STM32 主动周期上报。

seq:
    seq=0 表示 STM32 主动周期上报。
    seq!=0 表示对某条 LIFT_CONTROL 的任务反馈。
```

这个 topic 后续会给行为树、机械臂动作选择、调试界面使用。

---

### 5. `/r2_chassis/estop`

```text
service: /r2_chassis/estop
type: techx_r2_chassis_interfaces/srv/EStop
```

用于急停。

调用后，上位机会立即向 STM32 发送 `ESTOP` 串口帧。

---

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

frame_type：

```text
0x01 = Host -> STM32
0x02 = STM32 -> Host
```

第一版 cmd_type：

```text
0x30 = CHASSIS_VEL_CMD
0x31 = LIFT_CONTROL
0x32 = STEP_COMMAND

0x41 = LIFT_STATUS
0x42 = STEP_STATUS

0xE0 = ESTOP
```

---

### `CHASSIS_VEL_CMD`

上位机周期发送，不等待 ACK。

```text
cmd_type = 0x30
方向：Host -> STM32
```

data 区：

```text
data0~1: vx_mm_s，int16，小端
data2~3: vy_mm_s，int16，小端
data4~5: omega_mrad_s，int16，小端
data6: mode，第一版填 0
data7: reserved，填 0
```

ROS 2 单位换算：

```text
linear.x  m/s   -> vx_mm_s
linear.y  m/s   -> vy_mm_s
angular.z rad/s -> omega_mrad_s
```

---

### `LIFT_CONTROL`

上位机发送可靠任务，需要 STM32 返回 `LIFT_STATUS` 的 ACK / RUNNING / DONE / ERROR。

```text
cmd_type = 0x31
方向：Host -> STM32
```

第一版 data 区：

```text
data0~1: target_h_mm，int16，小端
data2: mask
data3~7: reserved，填 0
```

mask 定义：

```text
bit0 = 升降1
bit1 = 升降2
bit2 = 升降3

0x01 = 只控制升降1
0x02 = 只控制升降2
0x04 = 只控制升降3
0x07 = 三个升降一起控制
```

示例：三个升降一起到 200mm

```text
target_h_mm = 200 = 0x00C8
mask = 0x07

data:
C8 00 07 00 00 00 00 00
```

示例：只控制升降1到 100mm

```text
target_h_mm = 100 = 0x0064
mask = 0x01

data:
64 00 01 00 00 00 00 00
```

---

### `LIFT_STATUS`

STM32 返回升降状态。

```text
cmd_type = 0x41
方向：STM32 -> Host
```

data 区：

```text
data0~1: lift1_mm，int16，小端
data2~3: lift2_mm，int16，小端
data4~5: lift3_mm，int16，小端
data6: state
data7: error_code
```

第一版 state：

```text
1 = ACK
2 = RUNNING
3 = DONE
4 = ERROR
```

第一版 error_code：

```text
0x00 = OK
0x08 = MOTOR_ERROR
```

`LIFT_STATUS` 有两种用途：

```text
1. 作为 LIFT_CONTROL 的任务反馈：
   seq = 原 LIFT_CONTROL 的 seq

2. 作为 STM32 主动周期状态上报：
   seq = 0
```

例如：

```text
LIFT_CONTROL seq=8 target_h=200 mask=0x07

STM32 任务反馈：
    LIFT_STATUS seq=8 state=ACK
    LIFT_STATUS seq=8 state=RUNNING
    LIFT_STATUS seq=8 state=DONE

STM32 主动周期上报：
    LIFT_STATUS seq=0 lift1=200 lift2=200 lift3=200 state=DONE
```

---

### `STEP_COMMAND`

上位机发送可靠任务，需要 STM32 返回 `STEP_STATUS` 的 ACK / RUNNING / DONE / ERROR。

```text
cmd_type = 0x32
方向：Host -> STM32
```

data 区：

```text
data0: step_cmd
data1~2: delta_h_mm，int16，小端
data3: edge_id
data4: flags
data5~7: reserved，填 0
```

第一版 step_cmd：

```text
0 = MOVE_FLAT
1 = CLIMB_200
2 = CLIMB_400
3 = DESCEND_200
4 = DESCEND_400
```

---

### `STEP_STATUS`

STM32 返回台阶动作状态。

```text
cmd_type = 0x42
方向：STM32 -> Host
```

data 区：

```text
data0: step_cmd
data1: state
data2: error_code
data3: progress
data4~7: reserved，填 0
```

第一版 state：

```text
1 = ACK
2 = RUNNING
3 = DONE
4 = ERROR
```

第一版 error_code：

```text
0x00 = OK
0x08 = MOTOR_ERROR
```

---

### `ESTOP`

```text
cmd_type = 0xE0
方向：Host -> STM32
```

data 区：

```text
data0~7: 全 0
```

用于急停。STM32 收到后应立即停止底盘、升降和爬阶动作。

---

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

source /opt/ros/humble/setup.bash

colcon build --packages-select \
  techx_r2_chassis_interfaces \
  techx_r2_chassis_control \
  --symlink-install

source install/setup.bash
```

如果刚修改过 action/msg，建议清理后重新编译：

```bash
cd ~/r2_chassis_control_ws

rm -rf build/ install/ log/

colcon build --packages-select \
  techx_r2_chassis_interfaces \
  techx_r2_chassis_control \
  --symlink-install

source install/setup.bash
```

检查 package 是否识别成功：

```bash
ros2 pkg list | grep techx_r2_chassis
```

正常应看到：

```text
techx_r2_chassis_control
techx_r2_chassis_interfaces
```

检查接口是否生成成功：

```bash
ros2 interface show techx_r2_chassis_interfaces/action/StepCommand
ros2 interface show techx_r2_chassis_interfaces/action/LiftControl
ros2 interface show techx_r2_chassis_interfaces/msg/LiftStatus
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
techx_r2_chassis_control test_lift_control_client
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

---

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

---

## 测试升降状态 `/r2_chassis/lift_status`

启动 mock 后，另开一个终端：

```bash
cd ~/r2_chassis_control_ws
source install/setup.bash

ros2 topic echo /r2_chassis/lift_status
```

正常情况下应该能看到 STM32/mock 周期主动上报的升降状态：

```text
seq: 0
is_task_feedback: false
lift1_mm: 0
lift2_mm: 0
lift3_mm: 0
avg_height_mm: 0
max_diff_mm: 0
state: 3
error_code: 0
```

注意：每一个新终端都必须执行：

```bash
source ~/r2_chassis_control_ws/install/setup.bash
```

否则可能出现：

```text
The message type 'techx_r2_chassis_interfaces/msg/LiftStatus' is invalid
```

---

## 测试升降控制 `/r2_chassis/lift_control`

### 三个升降一起到 200mm

```bash
cd ~/r2_chassis_control_ws
source install/setup.bash

ros2 run techx_r2_chassis_control test_lift_control_client \
  --target-h-mm 200 \
  --mask 0x07 \
  --timeout-sec 10.0
```

正常流程：

```text
Action goal 发送
chassis_serial_node 进入 TaskMode
发送 0 速度
发送 LIFT_CONTROL
mock 返回 LIFT_STATUS ACK
mock 返回 LIFT_STATUS RUNNING
mock 返回 LIFT_STATUS DONE
Action 返回 success=true
chassis_serial_node 回到 NormalVelocity
```

同时 `/r2_chassis/lift_status` 应显示：

```text
lift1_mm: 200
lift2_mm: 200
lift3_mm: 200
avg_height_mm: 200
max_diff_mm: 0
```

### 只控制升降1到 100mm

```bash
ros2 run techx_r2_chassis_control test_lift_control_client \
  --target-h-mm 100 \
  --mask 0x01 \
  --timeout-sec 10.0
```

### 只控制升降2到 100mm

```bash
ros2 run techx_r2_chassis_control test_lift_control_client \
  --target-h-mm 100 \
  --mask 0x02 \
  --timeout-sec 10.0
```

### 只控制升降3到 100mm

```bash
ros2 run techx_r2_chassis_control test_lift_control_client \
  --target-h-mm 100 \
  --mask 0x04 \
  --timeout-sec 10.0
```

---

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
mock 返回 STEP_STATUS ACK
mock 返回 STEP_STATUS RUNNING
mock 返回 STEP_STATUS DONE
Action 返回 success=true
chassis_serial_node 回到 NormalVelocity
```

---

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

---

## 测试 ERROR

启动 mock 时打开错误模拟：

```bash
ros2 launch techx_r2_chassis_control mock_chassis.launch.py simulate_error:=true
```

然后发送爬阶动作：

```bash
ros2 run techx_r2_chassis_control test_step_command_client CLIMB_200 \
  --current-h 0 \
  --target-h 200 \
  --edge-id 1
```

或者发送升降动作：

```bash
ros2 run techx_r2_chassis_control test_lift_control_client \
  --target-h-mm 200 \
  --mask 0x07 \
  --timeout-sec 10.0
```

预期结果：

```text
mock 返回 ACK
mock 返回 RUNNING
mock 返回 ERROR
Action 返回 success=false
error_code = 0x08
```

---

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

如果后续 mock 也支持对 LIFT_CONTROL 丢弃第一次 ACK，也可以用同样方式测试升降任务重发。

---

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

---

## STM32 下位机需要实现的第一版协议

STM32 至少需要识别：

```c
case 0x30:
    handle_chassis_vel_cmd(seq, data);
    break;

case 0x31:
    handle_lift_control(seq, data);
    break;

case 0x32:
    handle_step_command(seq, data);
    break;

case 0xE0:
    handle_estop(seq);
    break;
```

### `CHASSIS_VEL_CMD`

```text
收到后解析 vx / vy / omega。
普通模式下直接更新底盘目标速度。
不需要每帧 ACK。
```

### `LIFT_CONTROL`

```text
收到后解析 target_h_mm 和 mask。
需要回 LIFT_STATUS ACK / RUNNING / DONE / ERROR。
```

伪代码：

```c
void handle_lift_control(uint8_t seq, uint8_t data[8])
{
    int16_t target_h_mm = read_i16_le(&data[0]);
    uint8_t mask = data[2];

    if (mask == 0 || (mask & ~0x07)) {
        send_lift_status(seq, STATE_ERROR, ERROR_INVALID_ID);
        return;
    }

    send_lift_status(seq, STATE_ACK, ERROR_OK);

    start_lift_move_to_height(target_h_mm, mask);
}
```

任务完成：

```c
send_lift_status(seq, STATE_DONE, ERROR_OK);
```

任务失败：

```c
send_lift_status(seq, STATE_ERROR, ERROR_MOTOR_ERROR);
```

### `LIFT_STATUS` 周期主动上报

STM32 建议 5Hz~10Hz 主动上报：

```c
send_lift_status(0, current_lift_state, current_error_code);
```

其中：

```text
seq = 0 表示主动周期上报
seq = 原命令 seq 表示任务反馈
```

发送优先级建议：

```text
最高：ESTOP / 严重 ERROR
第二：可靠任务 ACK / RUNNING / DONE / ERROR
第三：周期 LIFT_STATUS seq=0
```

### `STEP_COMMAND`

```text
收到后回 STEP_STATUS ACK / RUNNING / DONE / ERROR。
爬阶内部动作由 STM32 自己控制。
```

### `ESTOP`

```text
收到后立即停止底盘、升降、爬阶机构。
```

---

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
  ├── 发布 /cmd_vel
  ├── 调用 /r2_chassis/step_command
  └── 调用 /r2_chassis/lift_control 或订阅 /r2_chassis/lift_status

techx_R2_chassis_control
  ├── 订阅 /cmd_vel
  ├── 提供 /r2_chassis/step_command
  ├── 提供 /r2_chassis/lift_control
  ├── 发布 /r2_chassis/lift_status
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

---

## 行为树和机械臂如何使用升降状态

后续行为树可以使用：

```text
/r2_chassis/lift_control
```

来下发升降目标高度。

例如：

```text
target_h_mm = 200
mask = 0x07
```

表示三个升降一起到 200mm。

机械臂动作选择或行为树状态检查节点可以订阅：

```text
/r2_chassis/lift_status
```

然后根据：

```text
avg_height_mm
max_diff_mm
state
error_code
```

判断：

```text
1. 当前真实底盘高度是多少
2. 三个升降是否同步
3. 是否可以执行机械臂动作
4. 应该选择哪一套机械臂预设动作
```

例如：

```text
avg_height_mm 接近 0:
    选择低位动作

avg_height_mm 接近 200:
    选择 200mm 高度动作

max_diff_mm 超过阈值:
    禁止机械臂动作，等待升降恢复或报错
```

---

## 常用调试命令

查看 topic：

```bash
ros2 topic list
```

查看 `/cmd_vel`：

```bash
ros2 topic echo /cmd_vel
```

查看升降状态：

```bash
ros2 topic echo /r2_chassis/lift_status
```

查看 Action：

```bash
ros2 action list
ros2 action info /r2_chassis/step_command
ros2 action info /r2_chassis/lift_control
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

---

## 故障排查

### 1. 找不到 package

如果执行：

```bash
ros2 pkg list | grep techx_r2_chassis
```

只看到一个包，或者完全看不到，先确认是否已经 source：

```bash
source ~/r2_chassis_control_ws/install/setup.bash
```

如果还是不行，清理重编：

```bash
cd ~/r2_chassis_control_ws

rm -rf build/ install/ log/

colcon build --packages-select \
  techx_r2_chassis_interfaces \
  techx_r2_chassis_control \
  --symlink-install

source install/setup.bash
```

### 2. 新接口无法显示

如果执行：

```bash
ros2 interface show techx_r2_chassis_interfaces/action/LiftControl
ros2 interface show techx_r2_chassis_interfaces/msg/LiftStatus
```

报 `Unknown package`，通常是当前终端没有 source：

```bash
cd ~/r2_chassis_control_ws
source install/setup.bash
```

### 3. echo `/r2_chassis/lift_status` 报 message type invalid

如果看到：

```text
The message type 'techx_r2_chassis_interfaces/msg/LiftStatus' is invalid
```

一般是新终端没有 source 工作空间，或者 ROS 2 daemon 缓存旧接口。

解决：

```bash
cd ~/r2_chassis_control_ws
source install/setup.bash

ros2 daemon stop
ros2 daemon start

ros2 topic echo /r2_chassis/lift_status
```

每一个新终端都要执行：

```bash
source ~/r2_chassis_control_ws/install/setup.bash
```

### 4. 找不到节点

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

### 5. 串口打不开

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

### 6. mock launch 启动失败，提示找不到 socat

安装 socat：

```bash
sudo apt install -y socat
```

### 7. mock launch 启动后串口仍然打不开

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

### 8. 编译出现 CMP0148 warning

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

---

## 后续扩展方向

第一版已经支持：

```text
CHASSIS_VEL_CMD
LIFT_CONTROL
STEP_COMMAND
LIFT_STATUS
STEP_STATUS
ESTOP
```

后续可以逐步扩展：

```text
CHASSIS_STATUS
GYM_ALIGN
完整 CLIMB_STATUS
UNSYNC
LIMIT_TRIGGERED
ACTION_TIMEOUT
ESTOP_ACTIVE
更完整 error_code
LIFT_STOP
更复杂的升降速度控制
```

建议扩展顺序：

```text
1. 先稳定 /cmd_vel 速度流
2. 稳定 STEP_COMMAND 的 ACK / RUNNING / DONE / ERROR
3. 稳定 LIFT_CONTROL 的 ACK / RUNNING / DONE / ERROR
4. 稳定 LIFT_STATUS 周期状态上报
5. 接入真实 STM32
6. 接行为树 ChassisStepCommandNode 和 LiftControlNode
7. 用 /r2_chassis/lift_status 支持机械臂动作选择
8. 最后扩展底盘状态、对准、完整错误诊断
```

---

## 设计原则

```text
行为树不直接操作串口
导航只发布 /cmd_vel
ChassisStepCommandNode 只调用 /r2_chassis/step_command
LiftControlNode 只调用 /r2_chassis/lift_control
机械臂动作选择节点订阅 /r2_chassis/lift_status
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
STEP_STATUS / LIFT_STATUS 反馈
  ↓
ROS 2 Action 结果 / Topic 状态
```

这条闭环稳定跑通。