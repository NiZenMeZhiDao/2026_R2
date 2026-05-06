# 全自动机器人 ROS 2 三层架构设计

## 1. 设计目标

目标不是把系统拆得很复杂，而是让多人协作时边界清楚、接口清楚、流程容易调试。

机器人主流程：

```text
启动
  -> 导航到台阶
  -> 上/下台阶
  -> 导航到目标物体
  -> 识别物体
  -> 抓取
  -> 导航到协作区域
  -> 与另一台机器人配合抬升/拼装
  -> 放置
  -> 结束
```

推荐分成三层：

```text
任务层 Mission Layer
  决定现在做什么，负责流程跳转

能力层 Skill Layer
  完成一个完整动作，例如导航、上台阶、抓取、协作抬升

设备/状态层 Device & State Layer
  负责硬件控制、传感器数据、状态缓存
```

## 2. 三层结构

### 2.1 任务层 Mission Layer

负责整车流程。

推荐包：

```text
robot_runtime/
  mission_manager.py
```

职责：

- 维护整车状态机。
- 决定当前执行哪个能力。
- 根据成功、失败、超时、跳过决定下一步。
- 处理急停、暂停、恢复。

任务层不直接控制底盘、升降轮、机械臂。

示例状态：

```text
IDLE
NAV_TO_STAIRS
CLIMB_STAIRS
NAV_TO_OBJECT
DETECT_OBJECT
PICK_OBJECT
NAV_TO_ASSEMBLY_AREA
COOPERATIVE_LIFT
PLACE_OBJECT
DONE
ERROR
```

### 2.2 能力层 Skill Layer

负责完成一个完整动作。

推荐包：

```text
robot_runtime/
  skills/
    navigate_skill.py
    climb_stair_skill.py
    pick_object_skill.py
    place_object_skill.py
    cooperative_lift_skill.py
```

职责：

- 一个 skill 表示一个完整能力。
- 可以组合多个设备控制器。
- 可以读取当前机器人状态。
- 返回统一执行结果。
- 不直接写下位机协议。

例子：

```text
NavigateSkill
  使用底盘控制器，完成导航到目标点

ClimbStairSkill
  使用底盘 + 升降轮，完成上/下台阶

PickObjectSkill
  使用目标识别结果 + 底盘微调 + 机械臂 + 夹爪，完成抓取

CooperativeLiftSkill
  使用对方机器人状态 + 底盘 + 升降轮，完成协作抬升
```

### 2.3 设备/状态层 Device & State Layer

负责传感器、硬件控制和状态缓存。

推荐包：

```text
robot_runtime/
  robot_context.py
  robot_body.py
  controllers/
    chassis_controller.py
    suspension_controller.py
    arm_controller.py
    gripper_controller.py

robot_hardware/
  hardware_bridge_node.py

block_detection/
  object_detection_node.py

active_suspension_control/
  suspension_node.py
```

职责：

- `RobotContext` 缓存当前状态。
- `RobotBody` 统一持有所有 controller。
- controller 是硬件唯一控制入口。
- 外部 ROS 节点负责发布传感器和识别结果。

推荐 controller：

```text
ChassisController
  stop()
  move_to_pose(name)
  align_to_target(pose)
  slow_forward(speed)

SuspensionController
  stop()
  set_all_height(height)
  set_front_height(height)
  set_rear_height(height)
  wait_height_reached()

ArmController
  stop()
  home()
  move_to_pre_grasp(pose)
  move_to_grasp(pose)
  lift()
  move_to_place(pose)

GripperController
  stop()
  open()
  close()
```

## 3. 三层之间的接口

### 3.1 调用方向

只允许单向调用：

```text
MissionManager
  -> Skill
    -> RobotBody
      -> Controller
        -> ROS topic / action / service
```

反馈方向：

```text
sensor / hardware / perception node
  -> ROS topic
    -> robot_runtime_node callback
      -> RobotContext
        -> Skill / Controller 读取
```

不要反向调用：

```text
Controller 不调用 MissionManager
Skill 不直接订阅 ROS topic
视觉节点不直接控制机械臂
硬件节点不决定任务流程
```

### 3.2 RobotContext 接口

`RobotContext` 是共享状态缓存。

建议字段：

```python
class RobotContext:
    robot_pose = None
    nav_status = "idle"

    object_detected = False
    object_pose = None
    object_confidence = 0.0

    step_detected = False
    step_height = 0.0
    distances = []
    pe_switches = []
    wheel_heights = []

    arm_status = "idle"
    gripper_status = "idle"

    partner_online = False
    partner_state = "unknown"

    emergency_stop = False
```

规则：

- ROS callback 负责更新 `RobotContext`。
- skill 和 controller 主要读取 `RobotContext`。
- 新增字段要统一命名，避免每个人随便加。

### 3.3 RobotBody 接口

`RobotBody` 是统一硬件入口。

```python
class RobotBody:
    def __init__(self, chassis, suspension, arm, gripper):
        self.chassis = chassis
        self.suspension = suspension
        self.arm = arm
        self.gripper = gripper

    def stop_all(self):
        self.chassis.stop()
        self.suspension.stop()
        self.arm.stop()
        self.gripper.stop()
```

这样 skill 不直接保存一堆硬件对象，而是通过 `body.xxx` 调用。

## 4. Skill 执行接口

为了支持可变流程、超时、跳过、失败恢复，skill 推荐统一接口：

```python
class Skill:
    def start(self, context, body, goal=None):
        pass

    def tick(self, context, body):
        pass

    def stop(self, body):
        pass
```

统一返回：

```python
class SkillStatus:
    RUNNING = "running"
    SUCCESS = "success"
    FAILED = "failed"
    TIMEOUT = "timeout"
    SKIPPED = "skipped"
    CANCELED = "canceled"


class SkillResult:
    def __init__(self, status, message="", next_state=None):
        self.status = status
        self.message = message
        self.next_state = next_state
```

含义：

```text
RUNNING   当前能力还在执行
SUCCESS   完成，进入默认下一流程
FAILED    失败，进入恢复或错误流程
TIMEOUT   超时，进入恢复或重试
SKIPPED   条件不满足，跳过
CANCELED  被急停、暂停、人工接管打断
```

## 5. MissionManager 调用方式

`MissionManager` 是唯一决定流程跳转的地方。

伪代码：

```python
class MissionManager:
    def __init__(self, context, body, skills):
        self.context = context
        self.body = body
        self.skills = skills
        self.state = "IDLE"
        self.active_skill = None

    def tick(self):
        if self.context.emergency_stop:
            self.cancel_current()
            self.body.stop_all()
            self.state = "ERROR"
            return

        if self.state == "NAV_TO_STAIRS":
            self.run_skill(
                skill=self.skills.navigate,
                goal="stairs",
                default_next="CLIMB_STAIRS",
                fail_next="ERROR",
            )

        elif self.state == "CLIMB_STAIRS":
            self.run_skill(
                skill=self.skills.climb_stair,
                goal={"mode": "up"},
                default_next="NAV_TO_OBJECT",
                fail_next="RECOVER_STAIRS",
            )

        elif self.state == "PICK_OBJECT":
            self.run_skill(
                skill=self.skills.pick_object,
                goal=None,
                default_next="NAV_TO_ASSEMBLY_AREA",
                fail_next="RECOVER_OBJECT",
            )
```

统一执行函数：

```python
def run_skill(self, skill, goal, default_next, fail_next):
    if self.active_skill is None:
        result = skill.start(self.context, self.body, goal)
        self.active_skill = skill
    else:
        result = skill.tick(self.context, self.body)

    if result.status == SkillStatus.RUNNING:
        return

    self.active_skill = None

    if result.status in [SkillStatus.SUCCESS, SkillStatus.SKIPPED]:
        self.state = result.next_state or default_next
        return

    if result.status in [SkillStatus.FAILED, SkillStatus.TIMEOUT, SkillStatus.CANCELED]:
        skill.stop(self.body)
        self.state = result.next_state or fail_next
        return
```

这样某个流程如何退出就很清楚：

```text
SUCCESS  -> 默认下一流程
SKIPPED  -> 默认下一流程或指定 next_state
FAILED   -> 恢复流程或错误流程
TIMEOUT  -> 恢复流程或重试流程
CANCELED -> 停止/人工接管/错误流程
```

## 6. ROS 2 数据流

### 6.1 输入数据

```text
livox_ros_driver2
  -> /livox/lidar

wit_ros2_imu
  -> /imu/data

block_detection
  -> /object/detected
  -> /object/pose

robot_hardware / active_suspension_control
  -> /sensor_distances
  -> /r0x0201
  -> /suspension/status

robot_coordination
  -> /partner/status
```

`robot_runtime_node` 订阅这些 topic，然后更新 `RobotContext`。

### 6.2 输出命令

controller 负责发布命令：

```text
ChassisController
  -> /cmd_vel
  -> Nav2 action，可选

SuspensionController
  -> /t0x0102_action
  -> /suspension/cmd，可选

ArmController
  -> /arm/task_cmd
  -> /arm/target_pose

GripperController
  -> /gripper/cmd
```

### 6.3 状态输出

为了调试，`robot_runtime_node` 应发布：

```text
/mission/state
/mission/active_skill
/mission/status
/runtime/error
```

各模块也发布自己的状态：

```text
/nav/status
/suspension/status
/arm/status
/gripper/status
/coordination/status
```

## 7. Action 使用建议

Action 适合放在能力层，不适合放在底层控制命令。

适合做 Action：

```text
NavigateToNamedPose
ClimbStair
PickObject
PlaceObject
CooperativeLift
```

不适合做 Action：

```text
set_wheel_height
open_gripper
close_gripper
publish_cmd_vel
read_sensor
```

推荐路线：

```text
第一版：
  skill 是本地 Python 类
  MissionManager 直接调用 skill

稳定后：
  把部分 skill 包成 ROS 2 ActionServer

最终：
  MissionManager 可以调用本地 skill，也可以调用远程 action
```

Action 和 skill 的对应关系：

```text
Action Goal
  -> skill.start(context, body, goal)

Action Feedback
  -> skill.tick() 中的 phase/progress/message

Action Cancel
  -> skill.stop(body)

Action Result
  -> SkillResult
```

## 8. 多人协作分工

按三层分工：

```text
任务层负责人
  mission_manager.py
  负责整车流程、状态跳转、失败恢复

能力层负责人
  skills/*.py
  每个人负责一个完整能力

设备/状态层负责人
  controllers/*.py
  robot_context.py
  robot_body.py
  各硬件负责人维护自己的 controller
```

更具体：

```text
导航同学：
  ChassisController
  NavigateSkill

悬挂/台阶同学：
  SuspensionController
  ClimbStairSkill

视觉同学：
  block_detection
  RobotContext 中 object 字段

机械臂同学：
  ArmController
  GripperController
  PickObjectSkill
  PlaceObjectSkill

双机器人协作同学：
  robot_coordination
  CooperativeLiftSkill

runtime 负责人：
  runtime_node.py
  robot_context.py
  robot_body.py
  SkillResult / SkillStatus

整车流程负责人：
  mission_manager.py
```

协作规则：

- controller 只写基础硬件能力。
- skill 只写完整动作流程。
- mission 只写流程跳转。
- 传感器和识别只更新状态，不直接控制硬件。
- 新增 `RobotContext` 字段要统一命名。

## 9. 推荐第一版文件结构

```text
robot_runtime/
  robot_runtime/
    runtime_node.py
    mission_manager.py
    robot_context.py
    robot_body.py

    common/
      skill_result.py
      mission_state.py

    controllers/
      chassis_controller.py
      suspension_controller.py
      arm_controller.py
      gripper_controller.py

    skills/
      navigate_skill.py
      climb_stair_skill.py
      pick_object_skill.py
      place_object_skill.py
      cooperative_lift_skill.py

  config/
    mission.yaml
```

其他包保持清晰：

```text
robot_bringup/
  启动所有节点

robot_hardware/
  上下位机通信

active_suspension_control/
  现有悬挂控制，可逐步接入 SuspensionController

block_detection/
  目标识别

robot_coordination/
  双机器人状态通信
```

## 10. 总结

三层结构：

```text
任务层：
  MissionManager
  负责流程

能力层：
  Skill
  负责完整动作

设备/状态层：
  RobotBody + Controller + RobotContext
  负责硬件控制和状态缓存
```

最重要的边界：

```text
MissionManager 不直接控硬件
Skill 不直接订阅 topic
Controller 不决定任务流程
RobotContext 只存状态
```

这样结构简单，接口也比较稳定，后续再把成熟的 skill 包成 ROS 2 Action 即可。

