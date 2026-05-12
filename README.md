# 2026_R2

ROS 2 workspace for an autonomous mobile robot with:

- chassis motion control
- active suspension step climbing
- USB lower-machine bridge
- distance sensor input
- Odin SLAM/localization bridge

This repository is organized around a runtime-centered control path:
external drivers publish ROS topics, `robot_runtime` caches state and exposes
Python-call APIs, and upper-layer tasks or missions drive the robot by calling
those APIs.

## Main packages

### `robot_runtime`

Current bottom-layer runtime package.

- owns the shared `RobotContext`
- publishes chassis and suspension commands
- provides `RuntimeCore` as the main code-call API
- contains suspension math and step-climb task helpers

Detailed usage is documented in [robot_runtime/README.md](robot_runtime/README.md).

### `ares_usb`

USB bridge package for communicating with the lower machine.

### `multi_serial_sensor`

Distance sensor input package.

### `slam_odin_bridge`

Bridge package for treating `slam_odin/src/odin_ros_driver` as an external
localization driver. It converts Odin odometry plus TF into `/robot_pose` in the
`map` frame, republishes Odin IMU to `/imu/data`, and can publish `map.pcd` as
`/odin1/map`.

### `active_suspension_control`

Older standalone active suspension package kept here as reference during migration.

It should not run at the same time as `robot_runtime` suspension output,
otherwise both may publish to `/t0x0102_action`.

## Repository structure

```text
2026_R2/
├── robot_runtime/               # current runtime and task-side control entry
├── ares_usb/                    # USB bridge to lower machine
├── multi_serial_sensor/         # distance sensor driver
├── slam_odin_bridge/            # Odin external driver bridge
├── slam_odin/                   # Odin driver workspace and local maps
├── active_suspension_control/   # legacy suspension package for comparison
├── ros_architecture_design.md   # higher-level architecture notes
└── design.ini                   # local design notes, ignored in git
```

## Control architecture

Recommended layering in this workspace:

```text
Mission / top-level flow
  decides what to do next

Task / skill layer
  reads runtime state
  calls RuntimeCore / task helpers
  owns control loops

robot_runtime
  updates RobotContext from ROS topics
  publishes one-shot control commands
  provides reusable math and task helpers
```

The detailed Chinese design notes are in [ros_architecture_design.md](ros_architecture_design.md).

## Common topics

Input topics used by `robot_runtime`:

- `/sensor_distances`
- `/r0x0201`
- `/current_state`
- `/suspension/status`
- `/robot_pose`
- `/robot_pose_odom`
- `/imu/data`
- `/nav/status`
- `/localization/status`
- `/direction`
- `/emergency_stop`

Output topics:

- `/t0x0101_cmdvel`
- `/t0x0102_action`

## Build

From the workspace root:

```bash
colcon build
source install/setup.bash
```

If you only want to build the main runtime path first:

```bash
colcon build --packages-select robot_runtime slam_odin_bridge ares_usb multi_serial_sensor
source install/setup.bash
```

If you want to start Odin together with the runtime, also build the Odin driver:

```bash
colcon build --packages-select robot_runtime slam_odin_bridge odin_ros_driver ares_usb multi_serial_sensor
source install/setup.bash
```

## Run

Start the bottom-layer runtime stack:

```bash
ros2 launch robot_runtime runtime_bottom_layer.launch.py
```

Start the bottom-layer runtime stack with the Odin external driver bridge:

```bash
ros2 launch robot_runtime runtime_with_odin.launch.py \
  pcd_path:=/home/xiexiang/2026_R2/slam_odin/map.pcd \
  debug_period_sec:=0.5
```

`runtime_with_odin.launch.py` starts the lower-machine bridge, distance sensor,
`robot_runtime`, `odin_ros_driver`, `slam_odin_bridge`, and the static PCD map
publisher.

Run the preset navigation plus three-direction step-climb test entry:

```bash
ros2 run robot_runtime step_climb_forward
```

## Development notes

- `robot_runtime` is the main package to extend.
- controller modules should only publish hardware commands; they should not own task logic.
- sustained behaviors should live in task helpers or upper-layer mission code.
- the active suspension logic has been migrated into `robot_runtime/libraries/suspension_math.py`.

## Upload notes

This repository includes a root `.gitignore` for:

- `build/`, `install/`, `log/`
- Python cache files
- editor metadata
- local Codex metadata

Before pushing to GitHub, make sure the workspace root does not include private
machine-specific files you still want to keep local.
# 2026_R2
