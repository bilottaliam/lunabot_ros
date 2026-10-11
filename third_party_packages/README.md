# Third Party Packages

This directory contains third-party ROS2 packages integrated directly into the repository.

## Packages

### livox_ros_driver2
- **Description**: ROS2 driver for Livox LiDAR sensors
- **Original source**: https://github.com/Livox-SDK/livox_ros_driver2
- **Notes**: Modified for ROS 2 (stripped ROS1 support, uses `launch/` directory)

### livox_laser_simulation_RO2 (removed)
- Gazebo Classic only plugin, which does not exist for Gazebo Jetty. Simulation now uses a gz-sim
  `gpu_lidar` sensor plus `lunabot_util/pointcloud_to_livox`, which republishes it as the Livox
  CustomMsg on `/livox/lidar`. See MIGRATION.md.

### Point-LIO
- **Description**: Point-LIO LiDAR-inertial odometry, publishes to `/lio_odom`
- **Branch**: `grid_map_ros2` (commit `155f46d`)
- **Original source**: https://github.com/LihanChen2004/Point-LIO
- **Notes**: Modified for ROS 2 (`const SharedPtr` callbacks, C++17), added configurable `odom_frame_id`/`base_frame_id` params, TF publishing disabled, using UKF from robot_localization instead

### rtabmap
- **Description**: RTAB-Map core library for real-time SLAM
- **Branch**: `0.23.13-lyrical` (unmodified upstream copy)
- **Original source**: https://github.com/introlab/rtabmap
- **Notes**: Built with `-DRTABMAP_SYNC_MULTI_RGBD=ON` for dual-camera support

### rtabmap_ros
- **Description**: ROS2 wrapper for RTAB-Map
- **Branch**: `0.23.13-lyrical` (unmodified upstream copy)
- **Original source**: https://github.com/introlab/rtabmap_ros
- **Notes**: Built with `-DRTABMAP_SYNC_MULTI_RGBD=ON` for dual-camera support