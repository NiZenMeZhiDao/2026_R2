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

By default the bridge can fall back to publishing odometry as an unaligned
runtime pose when no `map` transform is available. In that case
`/localization/status` reports `localized_unaligned`, and move targets are in
the current odometry frame rather than a relocalized map frame.

The current robot mounts the Odin device facing backward. The mounting
extrinsics live in `config/odin_mount.yaml`; tune `mount_base_to_odin_x/y/z`
and `mount_base_to_odin_yaw` there so `/robot_pose` reports the robot center
rather than the Odin sensor frame. The raw `/odin1/odometry` topic is still
left untouched.

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
