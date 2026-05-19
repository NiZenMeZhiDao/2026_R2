# robot_runtime 调用流程说明

`robot_runtime` 是当前机器人架构里的底层运行时包。它只负责把外部 ROS 驱动数据写入 `RobotContext`，并提供统一的代码函数调用入口给中层任务使用。

当前约定：

- 外部驱动包之间继续用 ROS 2 topic 通信。
- 中层任务和底层控制之间使用 Python 函数调用。
- 底层 controller 不写任务逻辑，不做计算。
- 中层任务调用 `RuntimeCore` skill；持续命令由 `RuntimeCore` 后台 tick 继续发布，新命令覆盖旧命令。
- 底盘移动、定位移动、悬挂高度和台阶模式统一通过 skill 调用；复杂计算放在计算库或 task helper 中。

## 分层关系

```text
顶层 Mission
  决定整车流程，不直接控制执行器

中层 Task / Skill
  读取 RuntimeCore.context
  调用计算库
  调用 RuntimeCore skill
  组合动作和判断任务结束

底层 robot_runtime
  runtime_node: 只和外部驱动 topic 通信
  runtime_core: 给中层提供异步 skill API
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

slam_odin_bridge / Odin 外部定位驱动桥接
  -> /robot_pose
  -> /robot_pose_odom
  -> /imu/data
  -> /localization/status
  -> runtime_node
  -> RobotContext
```

底层控制输出：

```text
中层任务
  -> RuntimeCore.move()
  -> RuntimeCore 后台 skill tick
  -> ChassisController
  -> /t0x0101_cmdvel
  -> ares_usb 动态透传
  -> 下位机 0x0101

中层任务
  -> RuntimeCore.set_stepmode(True, direction)
  -> RuntimeCore 后台 skill tick
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
  -> core.move(vx, vy, wz, time=0)
  -> RuntimeCore 后台 skill tick
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
  -> core.set_stepmode(True, direction)
  -> RuntimeCore 后台 skill tick
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
- `suspension_math`

中层一般只需要拿到 `RuntimeCore`，然后通过它读状态、调用 skill。

`robot_runtime/robot_context.py`

共享状态缓存。由 `runtime_node` 通过 `RuntimeCore.update_xxx()` 更新，中层和计算库读取。

定位相关字段：

- `robot_pose_map`: `map` 坐标系下的机器人位姿。
- `robot_pose`: 兼容旧接口，当前等同于 `robot_pose_map`。
- `robot_pose_odom`: `odom` 坐标系下的 Odin 连续里程计位姿。
- `robot_pose_map_xytheta`: `[x, y, theta]`，map 坐标系下的平面位姿摘要。
- `robot_pose_odom_xytheta`: `[x, y, theta]`，odom 坐标系下的平面位姿摘要。
- `robot_x`、`robot_y`、`robot_theta`: map 坐标系下任务层常用定位字段。
- `odom_x`、`odom_y`、`odom_theta`: odom 坐标系下连续里程计字段。
- `map_ready`: map 定位是否可用。
- `localization_ready`: 兼容字段，当前等同于 `map_ready`。
- `odom_ready`: odom 里程计是否已经收到。
- `robot_pose_receive_time`、`odom_pose_receive_time`: runtime 收到位姿的本机单调时间，用于超时保护。
- `localization_status`: `waiting_for_odom`、`odom_only`、`localized` 等定位状态。

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

底盘和定位：

```python
core.move(vx, vy, wz, time=0.0)
core.move_to(x, y, theta, timeout=10.0, frame='map')
core.stop()
```

`robot_runtime/config/runtime.yaml` 里可以调常用定位消费策略：

- `default_move_to_frame`: `core.move_to()` 未显式传 `frame` 时使用 `map` 还是 `odom`。
- `allow_move_to_odom_fallback`: map 不可用时，是否允许默认 map 目标自动改用 odom 位姿。
- `pose_timeout_sec`: 位姿多久没刷新就让 `move_to` 停车报错。
- `move_to_xy_tolerance`、`move_to_theta_tolerance`: 到点判定阈值。

悬挂：

```python
core.set_height(height)
core.set_stepmode(True, direction=0)
core.set_stepmode(False)
```

## 中层调用示例

持续移动，直到下一条移动命令覆盖：

```python
core.move(0.20, 0.0, 0.0, time=0.0)
```

这会做三件事：

```text
记录 active motion skill
  -> RuntimeCore 后台 timer 持续发布 [vx, vy, wz]
  -> 新的 move/move_to/stop 覆盖旧移动命令
```

移动指定时长后自动停止：

```python
core.move(0.20, 0.0, 0.0, time=2.0)
```

移动到 map 坐标系目标位姿：

```python
core.move_to(x=1.0, y=0.5, theta=0.0, frame='map')
```

只按当前 odom 局部坐标移动：

```python
core.move_to(x=1.0, y=0.5, theta=0.0, frame='odom')
```

`move_to()` 默认是同步 skill：到达目标点后才返回，默认 10 秒超时。需要后台模式时可以用：

```python
core.move_to(x=1.0, y=0.5, theta=0.0, wait=False)
```

悬挂保持高度：

```python
core.set_height(30.0)
```

进入台阶模式：

```python
core.set_stepmode(True, direction=0)   # 0 前进，1 左，-1 右
phase = core.last_suspension_result['phase']
```

上台阶任务 helper：

```python
from robot_runtime.tasks import StepClimbConfig, StepClimbTask

task = StepClimbTask(core, StepClimbConfig.forward(speed=0.2))
task.reset()

while running:
    result = task.tick((0.0, y_error, yaw_error), now=time.monotonic())
    if task.is_done():
        break
    time.sleep(task.config.control_period)
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
/robot_pose_odom
/imu/data
/nav/status
/localization/status
/direction
/emergency_stop
```

输出：

```text
/t0x0101_cmdvel
/t0x0102_action
/runtime/debug
```

不再使用 topic 做内部控制，例如不再通过 `/runtime/test_command` 或 `/relative_pose_error` 控制底盘。

`/runtime/debug` 是中文调试文本，包含底盘速度指令、四轮高度指令、上楼梯方向、当前 move_to 目标和位置误差、光电遮挡情况、四轮实时高度（即 `r0x0201` 第 5-8 个数据）、测距数据、定位状态和最近错误。`r0x0201` 第 5-8 个数据已经作为四轮实时高度显示，不再额外重复输出完整原始数组。现场调试可以直接看：

```bash
ros2 topic echo /runtime/debug
```

## 启动

```bash
source install/setup.bash
ros2 launch robot_runtime runtime_bottom_layer.launch.py
```

调试输出默认 0.5 秒发布一次。可以调慢或关闭：

```bash
ros2 launch robot_runtime runtime_bottom_layer.launch.py debug_period_sec:=1.0
ros2 launch robot_runtime runtime_bottom_layer.launch.py debug_period_sec:=0.0
```

如果要只使用 Odin 桥接已经存在的 `/odin1/odometry`、`/odin1/imu` 和 TF，构建 runtime 与桥接包即可：

```bash
colcon build --packages-select robot_runtime slam_odin_bridge ares_usb multi_serial_sensor
source install/setup.bash
```

如果要由 launch 一起启动 Odin 外部驱动，需要同时构建 `odin_ros_driver`：

```bash
colcon build --packages-select robot_runtime slam_odin_bridge odin_ros_driver ares_usb multi_serial_sensor
source install/setup.bash
```

带 Odin 外部定位驱动和桥接启动：

```bash
source install/setup.bash
ros2 launch robot_runtime runtime_with_odin.launch.py \
  pcd_path:=/home/xiexiang/2026_R2/slam_odin/map.pcd \
  debug_period_sec:=0.5
```

这个 launch 会同时启动：

```text
ares_usb/usb_bridge_node
multi_serial_sensor/multi_serial_node
robot_runtime/runtime_node
odin_ros_driver/host_sdk_sample
slam_odin_bridge/odin_localization_bridge
slam_odin_bridge/pcd_map_publisher
```

默认 launch 会启动：

```text
ares_usb
multi_serial_sensor
robot_runtime_node
```

运行三方向上台阶和预设导航目标测试脚本：

```bash
source install/setup.bash
ros2 run robot_runtime step_climb_forward
```

这个入口是一个简单顶层任务样板，直接顺序调用 skill：先 `core.move_to()` 到预设 map 目标点，再调用 `StepClimbTask` 上台阶，中间演示了 `core.set_stepmode(False)` 和 `core.set_height(30.0)`。后续复杂任务可以直接照着这个函数改成“先去哪里、关主动升降、再去哪里、再打开”等流程。

`active_suspension_control/suspension_node` 不默认启动，避免它和 `robot_runtime` 同时发布 `/t0x0102_action` 造成控制冲突。

## 协作约定

- 新增传感器字段先加到 `RobotContext`。
- 新增硬件控制先加 controller，但 controller 只做发布，不做计算。
- 新增任务逻辑放到中层或 `tasks/`。
- 新增纯计算放到 `libraries/`。
- 内部模块之间优先函数调用，只有外部驱动边界使用 ROS topic。
- 需要持续控制时，优先新增或复用 `RuntimeCore` skill；底层 controller 仍保持“一次调用只发一次 ROS 命令”。
