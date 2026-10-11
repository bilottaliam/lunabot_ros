# lunabot_util

This package contains various utility nodes.

## Source Files
- **bandwidth_monitor.py**: Monitors ROS topic bandwidth and publishes statistics.
- **image_compressor.py**: Compresses camera images for efficient network transport.
- **power_monitor.py**: Monitors power consumption and publishes power data.
- **topic_remapper.cpp**: (Simulation) Republishes `/diff_drive_controller/odom` as `/wheel_odom` and forwards `/cmd_vel` (Twist) to `/diff_drive_controller/cmd_vel` as a TwistStamped.
- **pointcloud_to_livox.cpp**: (Simulation) Converts the Gazebo lidar PointCloud2 (`/livox/lidar_PointCloud2`) into the Livox CustomMsg on `/livox/lidar` that the real MID-360 driver publishes.
- **livox_reorient.cpp**: Rotates the Livox lidar and IMU data into the base_link orientation using the URDF mount transform and drops points inside the robot body box.
