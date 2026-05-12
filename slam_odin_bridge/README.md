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

`/robot_pose` is only published as a map-frame pose after the bridge can
transform Odin odometry into `map`. Until then `/localization/status` reports
`odom_only`.

## Run

Bridge an already-running Odin driver:

```bash
ros2 launch slam_odin_bridge odin_bridge.launch.py \
  pcd_path:=/home/xiexiang/2026_R2/slam_odin/map.pcd
```

Start the Odin driver and the bridge together:

```bash
ros2 launch slam_odin_bridge odin_external_bringup.launch.py \
  pcd_path:=/home/xiexiang/2026_R2/slam_odin/map.pcd
```

