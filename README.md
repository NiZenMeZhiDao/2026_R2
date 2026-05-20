# 2026_R2

全自主移动机器人的 ROS 2 工作空间，支持底盘运动控制、主动悬挂上下台阶、USB 下位机通信、测距传感器输入、Odin SLAM/定位桥接等功能。

## 架构设计

仓库遵循三层控制架构（详见 `design.ini`）：

```text
顶层（任务/流程层）        →  整车流程编排，调用不同任务，读取传感器信息，不直接控制执行器
中层（任务/skill 层）      →  读取传感器，控制执行器，完成特定任务，调用计算库进行控制计算
底层（执行器控制层）        →  直接控制执行器，接收传感器数据，提供接口给上层调用
```

数据流：

```text
外部驱动包（传感器 / USB下位机）
   ↓ ROS 2 topics
robot_runtime_node（底层）
   ↓ RobotContext 缓存
RuntimeCore API（中层 skill 接口）
   ↓ Python 函数调用
任务层 / 顶层流程
```

`robot_runtime` 是这个数据流的核心：外部驱动通过 ROS topic 发布数据，`robot_runtime_node` 将这些数据缓存到 `RobotContext`，`RuntimeCore` 暴露 Python API 供上层任务/流程调用，上层通过对这些 API 的组合编排驱动机器人。

## 设计原则

- **自顶向下**：顶层决策"做什么"，中层实现"怎么做"，底层负责"执行"
- **软总线解耦**：外部驱动通过 ROS topic 与 runtime 通信，runtime 内部通过 Python 函数调用通信
- **独占控制**：底盘和悬挂控制器使用 `ExclusiveController` 模式，确保同一时刻只有一个逻辑调用方，防止指令竞争
- **地图/里程计自适应**：`move_to` 默认使用 map 帧，可在未定位时自动降级到 odom 帧
- **PID 参数文件化**：PID 增益通过 JSON 文件配置，支持运行时重载调参

## 主要包

### `robot_runtime` — 核心运行时

底层运行时包，是整个仓库的核心。内部按职责划分为四个子模块：

| 子模块 | 职责 | 关键文件 |
|--------|------|----------|
| 本体 | ROS 节点入口，订阅外部驱动 topic，缓存到 RobotContext | [runtime_node.py](robot_runtime/robot_runtime/runtime_node.py) |
| Core | 中层 skill API（move/move_to/set_height/set_stepmode/stop），提供任务层调用 | [runtime_core.py](robot_runtime/robot_runtime/runtime_core.py) |
| Controllers | 独占式硬件指令发布器，底层直接控制底盘和悬挂 | [chassis_controller.py](robot_runtime/robot_runtime/controllers/chassis_controller.py)、[suspension_controller.py](robot_runtime/robot_runtime/controllers/suspension_controller.py) |
| Libraries | 纯计算库：PID 控制器、主动悬挂 20 态状态机 | [pid.py](robot_runtime/robot_runtime/libraries/pid.py)、[suspension_math.py](robot_runtime/robot_runtime/libraries/suspension_math.py) |
| Tasks | 任务层工具：StepClimbTask 实现底盘移动 + 悬挂爬台阶协调 | [step_climb_task.py](robot_runtime/robot_runtime/tasks/step_climb_task.py) |
| Config | Context 状态缓存、PID 参数加载、RobotBody 聚合 | [robot_context.py](robot_runtime/robot_runtime/robot_context.py)、[pid_config.py](robot_runtime/robot_runtime/pid_config.py) |

**对外暴露的 skill API**（`RuntimeCore` 提供）：

- `move(vx, vy, wz, time)` — 异步移动，time=0 表示持续直到收到新指令
- `move_to(x, y, theta, ...)` — 移动到目标位姿并旋转到目标角度，默认阻塞等待
- `stop()` — 停止所有 skill 并发布停止指令
- `set_height(height)` — 设置四轮悬挂高度
- `set_stepmode(enabled, direction)` — 开关台阶模式，direction: 0=前进, 1=左, -1=右

**两步启动与外部控制模式**：`RuntimeCore` 内置一个 200Hz skill 定时器用于定时刷新活跃的 motion / suspension skill，这样即使没有上层任务持续调用 tick，底层 skill 也能持续运行。当 `StepClimbTask` 等任务启动后，只需调用一次 `core.move(...)` 和 `core.set_stepmode(True, ...)`，skill 定时器将自动持续执行，无需上层反复循环调用。

**调试信息**：可通过 `/runtime/debug` topic 获得完整的中文运行时状态，包括定位状态、底盘速度指令、悬挂状态、传感器数据等。

### `slam_odin_bridge` — 定位桥接包

将外部的 Odin SLAM 驱动输出适配为 runtime 的标准 topic 格式：

- `/odin1/odometry` → `/robot_pose`（map 帧）+ `/robot_pose_odom`（odom 帧）
- `/odin1/imu` → `/imu/data`（透传重发）
- 发布 `/localization/status` 状态（`localized` / `odom_only` / `waiting_for_map`）
- 可选发布静态 PCD 地图到 `/odin1/map`

关键实现：[odin_localization_bridge.py](slam_odin_bridge/slam_odin_bridge/odin_localization_bridge.py) — 通过 TF 将 odom 帧里程计转换到 map 帧，支持异常时降级到里程计模式。

### `ares_usb` — USB 下位机桥接

C++ 实现的 USB 通信包，负责与下位机（ARES 机器人控制板）通信。发布 `r0x0201` 等 topic。关键实现位于 [usb_bridge_node.cpp](ares_usb/src/usb_bridge_node.cpp)，底层通信协议封装在 `ARES_bulk_library/` 中。

### `multi_serial_sensor` — 测距传感器

8 路串口测距传感器驱动，以 100Hz 频率从 `/dev/ttyCH9344USB0~7` 读取数据，过滤自信度=100 的有效测距点，以 `Float32MultiArray` 格式发布到 `/sensor_distances`。支持串口热插拔、自动重连、数据超时 NaN 处理。关键实现：[multi_serial_node.py](multi_serial_sensor/multi_serial_sensor/multi_serial_node.py)。

### `active_suspension_control` — 历史悬挂包

旧版独立悬挂控制包，在迁移过程中保留作为参考。**不应**与 `robot_runtime` 的悬挂输出同时运行，否则两者会同时发布到 `/t0x0102_action` 导致冲突。主动悬挂逻辑已迁移至 `robot_runtime/libraries/suspension_math.py`。

## 主动悬挂状态机

`suspension_math.py` 实现了一个 20 态的有限状态机，控制四轮独立悬挂完成上下台阶动作：

**上台阶流程**（10 个状态）：`IDLE → UP_1_PREPARE → UP_2_LIFT → UP_3_FRONT_DOCK → UP_4_RETRACT_FRONT → UP_5_FRONT_LAND → UP_6_SIDE_DOCK_RETRACT_REAR → UP_7_REAR_LAND → UP_8_RECOVER → IDLE`

**下台阶流程**（4 个状态）：`IDLE → DOWN_1_PREPARE → DOWN_2_FRONT_HOVER_LAND → DOWN_3_REAR_HOVER_LAND → DOWN_4_RECOVERY → IDLE`

状态机通过测距传感器检测台阶、光电开关（PE）检测轮子接触，利用稳定计数器（防抖）进行状态跳转。支持前进/左/右三个方向的虚拟传感器/执行器映射。

关键参数：
- `lift_low=205mm` / `lift_high=400mm`：两级抬升高度
- `height_tolerance=20mm`：高度到位容差
- `step_detect_threshold=200mm`：台阶检测距离阈值
- `idle_trigger_stable_ticks=2`：IDLE 态触发防抖

## 目录结构

```text
2026_R2/
├── robot_runtime/               # 核心运行时（三层架构的实现主体）
│   ├── robot_runtime/
│   │   ├── runtime_node.py      # ROS 节点入口，订阅外部驱动 topic
│   │   ├── runtime_core.py      # 中层 skill API
│   │   ├── robot_context.py     # 共享状态缓存（dataclass）
│   │   ├── robot_body.py        # 底盘+悬挂控制器聚合
│   │   ├── pid_config.py        # PID 参数文件加载
│   │   ├── debug_context.py     # 中文调试信息格式化
│   │   ├── step_climb_forward.py # 顶层任务示例（导航+上台阶）
│   │   ├── controllers/         # 独占式硬件指令发布器
│   │   ├── libraries/           # 纯计算库（PID、悬挂状态机）
│   │   └── tasks/               # 任务层工具（StepClimbTask）
│   ├── launch/                  # ROS 2 launch 文件
│   ├── config/                  # 运行参数配置
│   └── test/                    # 单元测试
├── slam_odin_bridge/            # Odin 外部定位驱动桥接
│   ├── slam_odin_bridge/
│   │   ├── odin_localization_bridge.py
│   │   ├── pcd_map_publisher.py
│   │   └── pose_math.py         # 四元数坐标变换工具
│   ├── launch/
│   └── config/
├── ares_usb/                    # USB 下位机桥接（C++）
├── multi_serial_sensor/         # 8路串口测距传感器（Python）
├── active_suspension_control/   # 旧版悬挂（仅作参考）
├── build/                       # colcon 构建产物
├── install/                     # colcon 安装产物
├── log/                         # 运行日志
├── design.ini                   # 架构设计笔记（gitignore）
└── .gitignore
```

## 核心 Topic

### 输入 topic（外部驱动 → runtime）

| Topic | 类型 | 来源 | 说明 |
|-------|------|------|------|
| `/sensor_distances` | Float32MultiArray | multi_serial_sensor | 8 路测距传感器原始值 |
| `/r0x0201` | Float32MultiArray | ares_usb | 下位机状态（光电、轮高、方向） |
| `/current_state` | Int32 | 悬挂驱动 | 悬挂状态机当前状态 |
| `/suspension/status` | String | 悬挂驱动 | 悬挂状态文本 |
| `/robot_pose` | PoseStamped | slam_odin_bridge | map 帧机器人位姿 |
| `/robot_pose_odom` | PoseStamped | slam_odin_bridge | odom 帧里程计位姿 |
| `/imu/data` | Imu | slam_odin_bridge | IMU 数据 |
| `/nav/status` | String | 导航模块 | 导航状态 |
| `/localization/status` | String | slam_odin_bridge | 定位状态（localized / odom_only） |
| `/direction` | Int32 | 下位机 | 方向指令（光电触发） |
| `/emergency_stop` | Bool | 急停按钮 | 急停信号 |

### 输出 topic（runtime → 执行器）

| Topic | 类型 | 说明 |
|-------|------|------|
| `/t0x0101_cmdvel` | Float32MultiArray | 底盘速度指令 `[vx, vy, wz]` |
| `/t0x0102_action` | Float32MultiArray | 悬挂高度指令 `[FL, FR, RL, RR]` |
| `/runtime/debug` | String | 中文调试状态汇总 |

## 构建

从工作空间根目录：

```bash
# 完整构建
colcon build
source install/setup.bash

# 仅构建核心运行时路径
colcon build --packages-select robot_runtime slam_odin_bridge ares_usb multi_serial_sensor
source install/setup.bash

# 含 Odin 驱动的完整构建
colcon build --packages-select robot_runtime slam_odin_bridge odin_ros_driver ares_usb multi_serial_sensor
source install/setup.bash
```

## 运行

启动底层运行时栈（下位机通信 + 测距传感器 + robot_runtime）：

```bash
ros2 launch robot_runtime runtime_bottom_layer.launch.py
```

启动完整栈（底层 + Odin SLAM + 定位桥接 + PCD 地图发布）：

```bash
ros2 launch robot_runtime runtime_with_odin.launch.py \
  pcd_path:=/home/xiexiang/2026_R2/slam_odin/map.pcd \
  debug_period_sec:=0.5
```

运行预设的导航+上台阶测试任务：

```bash
ros2 run robot_runtime step_climb_forward
```

## 配置

### 运行时参数

[runtime.yaml](robot_runtime/config/runtime.yaml) 可配置：

- `default_move_to_frame`: move_to 的目标帧（`map` 或 `odom`）
- `allow_move_to_odom_fallback`: map 可用时是否自动降级到 odom
- `pose_timeout_sec`: 定位数据超时阈值
- `move_to_xy_tolerance` / `move_to_theta_tolerance`: 位置/角度到位容差

### PID 参数

PID 增益通过 JSON 文件配置，放置在 `install/robot_runtime/share/robot_runtime/config/pid.json` 或通过环境变量 `ROBOT_RUNTIME_PID_CONFIG` 指定路径。默认内置参数（位于 `pid_config.py`）已可运行。配置结构：

```json
{
  "chassis_pose": { "x": {...}, "y": {...}, "yaw": {...} },
  "step_climb": { "y": {...}, "wz": {...} },
  "chassis_deadzone": { "xy": 0.12, "wz": 0.15 }
}
```

### 桥接参数

[slam_odin_bridge/config/param.yaml](slam_odin_bridge/config/param.yaml) 可配置 Odin topic 重映射、TF 帧名称、降级策略等。

## 测试

```bash
cd robot_runtime
python3 -m pytest test/ -v
```

测试覆盖：
- `test_suspension_math.py` — 悬挂状态机状态跳转逻辑
- `test_step_climb_task.py` — 上台阶任务的底盘速度计算、PID 校正、角度环绕
- `test_runtime_core_skills.py` — move/move_to/set_stepmode 等 skill API 行为
- `test_runtime_core_localization.py` — map/odom 帧选择与降级逻辑
- `test_pid_config.py` — PID 参数加载与合并
- `test_runtime_debug_context.py` — 调试信息格式化

## 开发指南

- **扩展新任务**：在 `robot_runtime/tasks/` 下添加新的任务类，参考 `step_climb_task.py` 的模式：初始化时传入 core 和 config → `tick()` 读取位姿误差 → 调用 core API 控制底盘和悬挂
- **添加新 skill**：在 `RuntimeCore` 中添加异步 skill 方法，并在 `tick_skills()` 中增加对应的驱动逻辑
- **控制器禁止直接操作**：控制器模块（`controllers/`）只负责发布硬件指令，不应包含任务逻辑；持续的闭环行为应放在 tasks 或 libraries 中
- **状态机调参**：悬挂状态机的稳定计数器阈值在 `SuspensionConfig` 和各阶段 `_is_stable` 的 threshold 参数中
- **PID 在线调参**：修改 JSON 配置文件后调用 `pid_config.reload_pid_config()` 即可生效，无需重启
- **不要同时运行两个悬挂控制器**：`active_suspension_control` 和 `robot_runtime` 的悬挂输出互斥

## 上传说明

仓库根目录 `.gitignore` 已忽略 `build/`、`install/`、`log/`、Python 缓存文件、IDE 元数据、以及 `design.ini`。推送前确认工作区根目录不包含需要保密的机器特定文件。
