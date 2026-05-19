# slam_odin_bridge

`slam_odin_bridge` adapts the external Odin ROS driver to this workspace's
runtime topic contract.

It keeps `odin_ros_driver` as an external sensor driver and publishes stable
topics that `robot_runtime` can cache in `RobotContext`.

## Data Flow

```text
odin_ros_driver
  -> /odin1/odometry
  -> /odin1/imu
  -> /tf odom <-> map

slam_odin_bridge
  -> /robot_pose       PoseStamped in map frame
  -> /robot_pose_odom  PoseStamped in odom frame
  -> /imu/data
  -> /localization/status
  -> /odin1/map        PointCloud2 static map
```

The bridge treats Odin odometry as the robot-center pose directly. It does not
apply any mounting offset, axis reversal, or yaw correction before publishing
`/robot_pose` and `/robot_pose_odom`.

When no `map` transform is available, `/localization/status` reports
`odom_only`. In that state the bridge keeps publishing `/robot_pose_odom`, but
does not pretend odometry is a relocalized `/robot_pose` unless map data really
exists.

Runtime behavior is configured in `config/param.yaml`:

- `allow_odom_fallback`: keep odom-only operation available when map TF is
  missing.
- `zero_pose_on_start`: when `false`, publish Odin/map coordinates directly;
  when `true`, publish poses relative to the first received pose.

## Run

Bridge an already-running Odin driver:

```bash
ros2 launch slam_odin_bridge odin_bridge.launch.py \
  pcd_path:=~/2026_R2/slam_odin/map.pcd
```

Start the Odin driver and the bridge together:

```bash
ros2 launch slam_odin_bridge odin_external_bringup.launch.py \
  pcd_path:=~/2026_R2/slam_odin/map.pcd
```
