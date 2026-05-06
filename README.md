# 2026_R2

ROS 2 workspace for an autonomous mobile robot with:

- chassis motion control
- active suspension step climbing
- USB lower-machine bridge
- distance sensor input

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
- `/imu/data`
- `/nav/status`
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
colcon build --packages-select robot_runtime ares_usb multi_serial_sensor
source install/setup.bash
```

## Run

Start the bottom-layer runtime stack:

```bash
ros2 launch robot_runtime runtime_bottom_layer.launch.py
```

Run the default forward step-climb task entry:

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
