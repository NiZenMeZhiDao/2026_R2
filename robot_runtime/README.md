# robot_runtime 调用流程说明

`robot_runtime` 是当前机器人架构里的底层运行时包。它只负责把外部 ROS 驱动数据写入 `RobotContext`，并提供统一的代码函数调用入口给中层任务使用。

当前约定：

- 外部驱动包之间继续用 ROS 2 topic 通信。
- 中层任务和底层控制之间使用 Python 函数调用。
- 底层 controller 不写循环逻辑，不做计算。
- 中层任务自己决定循环频率，每调用一次底层函数，底层只发送一次命令。
- 底盘 PID、悬挂状态机这类计算放在计算库或 task helper 中，由中层选择性调用。

## 分层关系

```text
顶层 Mission
  决定整车流程，不直接控制执行器

中层 Task / Skill
  读取 RuntimeCore.context
  调用计算库
  调用 RuntimeCore 控制函数
  自己维护任务循环

底层 robot_runtime
  runtime_node: 只和外部驱动 topic 通信
  runtime_core: 给中层提供 Python 函数 API
  controller: 调用一次，向 ROS topic 发布一次控制命令
```

## 数据流

外部驱动输入：

```text
multi_serial_sensor
  -> /sensor_distances
  -> runtime_node
  -> RuntimeCore.update_distances()
  -> RobotContext

ares_usb / 下位机
  -> /r0x0201
  -> runtime_node
  -> RuntimeCore.update_lower_machine()
  -> RobotContext
```

底层控制输出：

```text
中层任务
  -> RuntimeCore.set_chassis_velocity()
  -> ChassisController
  -> /t0x0101_cmdvel
  -> ares_usb 动态透传
  -> 下位机 0x0101

中层任务
  -> RuntimeCore.run_suspension_math_once()
  -> SuspensionMath.tick(context)
  -> SuspensionController
  -> /t0x0102_action
  -> ares_usb 动态透传
  -> 下位机 0x0102
```

### 底盘速度发布数据流

底盘速度命令使用 USB 透传 topic，不走 `geometry_msgs/Twist` 的 `/cmd_vel`。

```text
中层任务
  -> core.set_chassis_velocity(vx, vy, wz)
  -> ChassisController.set_velocity()
  -> 发布 std_msgs/Float32MultiArray 到 /t0x0101_cmdvel
  -> ares_usb 识别 t0x0101 前缀
  -> 透传到下位机 data_id 0x0101
```

消息内容：

```text
/t0x0101_cmdvel
type: std_msgs/msg/Float32MultiArray
data: [x方向速度, y方向速度, z轴旋转速度]
```

其中 `data[0]` 是 x 方向速度，`data[1]` 是 y 方向速度，`data[2]` 是 z 轴旋转速度。

### 四轮高度发布数据流

四轮主动抬升参考 `active_suspension_control` 包的发布方式：只发布四个轮子的目标高度。

```text
中层任务
  -> core.run_suspension_math_once()
  -> SuspensionMath.tick(context)
  -> 得到 wheel_targets = [h0, h1, h2, h3]
  -> SuspensionController.set_wheel_heights()
  -> 发布 std_msgs/Float32MultiArray 到 /t0x0102_action
  -> ares_usb 识别 t0x0102 前缀
  -> 透传到下位机 data_id 0x0102
```

消息内容：

```text
/t0x0102_action
type: std_msgs/msg/Float32MultiArray
data: [轮0目标高度, 轮1目标高度, 轮2目标高度, 轮3目标高度]
```

和 `active_suspension_control` 一样，`/t0x0102_action` 只表达四轮高度目标；是否继续循环、何时进入下一阶段由中层任务决定。

## 关键文件

`robot_runtime/runtime_node.py`

ROS 节点外壳，只订阅外部驱动 topic，然后调用 `RuntimeCore.update_xxx()` 更新 context。这里不写控制循环，不接内部测试命令。

`robot_runtime/runtime_core.py`

中层任务的主要调用入口。它持有：

- `context`
- `body`
- `chassis_pid`
- `suspension_math`

中层一般只需要拿到 `RuntimeCore`，然后通过它读状态、调用底层。

`robot_runtime/robot_context.py`

共享状态缓存。由 `runtime_node` 通过 `RuntimeCore.update_xxx()` 更新，中层和计算库读取。

`robot_runtime/robot_body.py`

统一持有底层 controller：

- `body.chassis`
- `body.suspension`

`robot_runtime/controllers/chassis_controller.py`

底盘控制发布器，只负责发布 `/t0x0101_cmdvel`。不做 PID，不做任务判断。

`robot_runtime/controllers/suspension_controller.py`

悬挂控制发布器，只负责发布 `/t0x0102_action`。不做台阶判断。

`robot_runtime/controllers/exclusive_controller.py`

controller 独占保护。调用 controller 时必须带 `owner`，避免多个任务同时抢同一个执行器。

`robot_runtime/libraries/pid.py`

通用 PID 计算库。

`robot_runtime/tasks/chassis_pid_task.py`

底盘相对位姿误差 PID 任务 helper。输入相对误差，输出 `Twist`。

`robot_runtime/libraries/suspension_math.py`

主动悬挂计算库。作用类似原 `active_suspension_control` 包里的状态机，但这里不直接发布 ROS topic，而是直接从 `RobotContext` 读取数据，计算四轮目标高度。

`robot_runtime/tasks/step_climb_task.py`

上台阶任务 helper。保持基础底盘速度，使用 PID 修正 `y` 和 `wz`，同时每周期调用悬挂状态机。

## RuntimeCore 常用 API

底盘：

```python
core.set_chassis_velocity(vx, vy, wz)
core.set_chassis_twist(twist)
core.pid_to_relative_pose(pose_error)
core.reset_chassis_pid()
```

悬挂：

```python
core.set_suspension_heights([h0, h1, h2, h3])
core.set_all_suspension_height(height)
core.run_suspension_math_once()
core.reset_suspension_math()
```

安全和占用：

```python
core.stop_all()
core.release_controllers()
```

## 中层调用示例

底盘 PID 微调：

```python
from geometry_msgs.msg import Twist

error = Twist()
error.linear.x = 0.20
error.linear.y = -0.03
error.angular.z = 0.10

cmd = core.pid_to_relative_pose(error)
```

这会做三件事：

```text
读取 error
  -> ChassisPidTask 计算 Twist
  -> ChassisController 转成 [vx, vy, wz]
  -> 发布 /t0x0101_cmdvel 一次
```

悬挂状态机走一步：

```python
result = core.run_suspension_math_once()
phase = result['phase']
targets = result['wheel_targets']
```

这会做三件事：

```text
读取 core.context 中的测距、PE、轮高、方向
  -> SuspensionMath.tick(context) 计算四轮目标高度
  -> SuspensionController 发布 /t0x0102_action 一次
```

中层如果要循环，需要自己控制频率：

```python
while running:
    result = core.run_suspension_math_once()
    if result['phase'].name == 'IDLE':
        break
    time.sleep(0.01)
```

上台阶任务 helper：

```python
from robot_runtime.tasks import StepClimbConfig, StepClimbTask

task = StepClimbTask(core, StepClimbConfig.forward(speed=0.2))
task.reset()

while running:
    result = task.tick((0.0, y_error, yaw_error), now=time.monotonic())
    if result['suspension']['phase'].name == 'IDLE':
        break
    time.sleep(0.01)
```

## Controller 独占规则

controller 防止多个任务同时控制同一执行器。底层实际调用时会使用 `owner`：

```python
body.chassis.set_velocity(0.1, 0.0, 0.0, owner='climb_task')
body.suspension.set_all_height(30.0, owner='climb_task')
```

如果 controller 已经被其他 owner 占用，会抛出 `ControllerBusyError`。

任务结束后释放：

```python
body.chassis.release('climb_task')
body.suspension.release('climb_task')
```

`RuntimeCore` 默认 owner 是 `runtime_core`。普通中层优先调用 `RuntimeCore` 的封装函数，不直接调用 controller。

## 外部 topic

runtime 只保留这些外部 topic：

输入：

```text
/sensor_distances
/r0x0201
/current_state
/suspension/status
/robot_pose
/imu/data
/nav/status
/direction
/emergency_stop
```

输出：

```text
/t0x0101_cmdvel
/t0x0102_action
```

不再使用 topic 做内部控制，例如不再通过 `/runtime/test_command` 或 `/relative_pose_error` 控制底盘。

## 启动

```bash
source install/setup.bash
ros2 launch robot_runtime runtime_bottom_layer.launch.py
```

默认 launch 会启动：

```text
ares_usb
multi_serial_sensor
robot_runtime_node
```

运行前进上台阶脚本：

```bash
source install/setup.bash
ros2 run robot_runtime step_climb_forward
```

`active_suspension_control/suspension_node` 不默认启动，避免它和 `robot_runtime` 同时发布 `/t0x0102_action` 造成控制冲突。

## 协作约定

- 新增传感器字段先加到 `RobotContext`。
- 新增硬件控制先加 controller，但 controller 只做发布，不做计算。
- 新增任务逻辑放到中层或 `tasks/`。
- 新增纯计算放到 `libraries/`。
- 内部模块之间优先函数调用，只有外部驱动边界使用 ROS topic。
- 需要持续控制时，中层自己写循环；底层每次函数调用只发布一次。
