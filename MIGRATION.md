# Migration: Humble / Gazebo Classic → Lyrical / Gazebo Jetty

Branch: `upgrade/lyrical`

## Targets

| | Before | After | Why |
|---|---|---|---|
| Ubuntu | 22.04 Jammy | **26.04 LTS Resolute** | Newest Ubuntu LTS (April 2026) |
| ROS 2 | Humble | **Lyrical Luth** | LTS distro whose REP-2000 Tier 1 platform is Ubuntu 26.04 (supported until May 2031) |
| Gazebo | Classic 11 | **Jetty (LTS)** | The pairing gazebosim.org recommends for Lyrical, shipped through the `gz_*_vendor` packages |
| Docker base | `osrf/ros:humble-desktop-full` | `osrf/ros:lyrical-desktop-full` | |

Sources: [ROS 2 Lyrical release announcement](https://discourse.openrobotics.org/t/ros-2-lyrical-luth-released/55021),
[REP 2000](https://www.ros.org/reps/rep-2000.html),
[Gazebo ROS installation / version pairing](https://gazebosim.org/docs/latest/ros_installation/).

## What changed

### Simulation (Gazebo Classic → gz-sim)
- **Launch** (`lunabot_bringup/launch/gui_launch.py`): `gazebo_ros/gazebo.launch.py` + `spawn_entity.py` replaced by
  `ros_gz_sim/gz_sim.launch.py` + `ros_gz_sim create`. `sim_gui:=false` runs `gz sim -s --headless-rendering` so the
  cameras and lidar still render. A `ros_gz_bridge` node is configured by
  `lunabot_config/params/gazebo/ros_gz_bridge.yaml` and bridges `/clock`, IMU, lidar and the three RGB-D cameras.
  The ROS topic names are the same as before.
- **URDF** (`v1_bot`, `v2_bot`):
  - `depth` + `libgazebo_ros_camera.so` → `rgbd_camera` sensors with `gz_frame_id` set to the same optical frames
  - `libgazebo_ros_imu_sensor.so` → gz `imu` sensor with the same 0.01 gaussian noise
  - `libgazebo_ros2_control.so` / `gazebo_ros2_control/GazeboSystem` → `gz_ros2_control-system` / `gz_ros2_control/GazeboSimSystem`
  - Removed the `Gazebo/Grey` material tags, which gz-sim ignores.
  - Wheel friction now uses the URDF `<gazebo>` extension tags `<mu1>/<mu2>/<kp>/<kd>`. The old `<mu>` and
    `<surface><ode>` children were never mapped by sdformat and were silently ignored. ODE `slip1/slip2` has no gz-sim
    equivalent and is dropped.
  - `ros2_control` command limits are now enforced. They were ignored under gazebo_ros2_control. Wheel velocity
    limits went from ±1 rad/s (0.15 m/s, below what Nav2 and the GUI command) to ±10 rad/s, and bucket position limits from ±0.75 to the
    joint's real range [-1.57, 0.1], so the robot behaves as it did on Humble.
- **Livox MID-360**: the vendored `livox_laser_simulation_RO2` plugin is Gazebo Classic only (it links `gazebo_dev`), and
  no gz-sim port exists. It is replaced by a `gpu_lidar` sensor (360 × 32 rays, same −0.126…0.964 rad vertical FOV and range)
  plus a new node, `lunabot_util/pointcloud_to_livox`. That node converts the bridged PointCloud2 into
  `livox_ros_driver2/CustomMsg` on `/livox/lidar`, so `livox_reorient` → Point-LIO / FAST-LIO see the same input as before.
  - Difference: the simulated lidar uses a regular grid instead of Livox's non-repetitive rosette pattern (the old plugin replayed `mid360.csv`), and
    all points share one timestamp (`offset_time = 0`). The old plugin's offsets were also close to 0.
- **Worlds** (`lunabot_sim/worlds/**`): OGRE `<script>` materials → PBR `albedo_map` textures. Gazebo/GreyTransparent becomes
  a grey diffuse color with `<transparency>`. Added the gz-sim Physics, UserCommands, SceneBroadcaster, Sensors (ogre2) and Imu
  system plugins. Fixed `<background>` colors given in 0–255, which sdformat now rejects. Removed the Classic `<gui>` camera.
  All 8 worlds pass `gz sdf -k`.
- **Model path**: the `gazebo_ros gazebo_model_path` export becomes an ament env hook that adds
  `share/lunabot_sim/models` to `GZ_SIM_RESOURCE_PATH`.

- **Controllers** (`lunabot_config/params/gazebo/*.yaml`, `gui_launch.py`):
  - `position_controllers/JointGroupPositionController` no longer exists in Lyrical. `position_controller` and
    `camera_controller` are now `forward_command_controller/ForwardCommandController` with `interface_name: position`.
    The `~/commands` topic and message type are unchanged, so the GUI needs no change.
  - Controllers no longer inherit the params file given to `gz_ros2_control`, so the spawner passes it with `--param-file`.
  - A single spawner now activates all controllers, with 120 s switch and service timeouts. Switches only complete inside a
    sim step, so with the Gazebo GUI open on a slow machine (about 7% real-time factor on the test laptop) the default 5 s timeout
    failed and left the robot with no drive controller.
  - Removed `use_stamped_vel` (no longer a diff_drive parameter).
  - Fixed `wheel_radius` 0.15 → 0.1016 to match the URDF wheels. This bug predates the migration (Humble had it too). With
    0.15 the robot drove at about 68% of the commanded speed and odometry over-reported distance.
- **`lunabot_gui`** now declares `python3-pyqt5` / `python3-opencv`, which were only installed by hand before.
- **`lunabot_gui` on Wayland**: Ubuntu 26.04 runs Qt5 apps on native Wayland. There the control panel got no keyboard
  input, so WASD teleop did nothing. Offscreen QTest checks showed the key handling itself is fine. `gui_launch.py` now
  sets `QT_QPA_PLATFORM=xcb` (XWayland) for the GUI unless the variable is already set.

### ROS 2 API changes
- `ament_target_dependencies()` was removed in Lyrical. The CMake files of `lunabot_nav`, `lunabot_teleop`, `lunabot_util`, `fast_lio` and `point_lio`
  now link `${<pkg>_TARGETS}` with `target_link_libraries`. `lunabot_teleop` was missing `find_package(ament_cmake)`, now added.
- The tf2 `.h` compatibility headers were removed, so includes now use `.hpp`.
- `diff_drive_controller` only accepts `TwistStamped` on `~/cmd_vel` (`cmd_vel_unstamped` and `use_stamped_vel` are gone).
  `topic_remapper` now stamps `/cmd_vel` and publishes to `/diff_drive_controller/cmd_vel`. `/cmd_vel` itself stays
  `geometry_msgs/Twist`, because the real-robot `controller_teleop`, the GUI and the sim action servers all use Twist, and Nav2's
  `controller_server` / `behavior_server` get `enable_stamped_cmd_vel: false` to match.

### Dependencies
| Dependency | Status on Lyrical | Resolution |
|---|---|---|
| `gazebo_ros_pkgs`, `gazebo_ros2_control` | Not released (Classic) | `ros_gz_sim`, `ros_gz_bridge`, `gz_ros2_control` |
| `rtabmap` / `rtabmap_ros` 0.21.9-humble (vendored) | Doesn't build | Re-vendored upstream tag `0.23.13-lyrical`. The old copy had no local changes (checked by diffing against upstream). `RTABMAP_SYNC_MULTI_RGBD` is still supported. |
| `grid_map` | Not released | rtabmap upstream already comments it out. Before the migration it was apt-removed after rosdep anyway, so rtabmap never used it and no feature is lost. |
| `libpointmatcher` (+ `libnabo`) | Not released | Built from source (norlab-ulaval) in the Dockerfile and `install_dependencies.sh`, because RTAB-Map uses `Icp/Strategy: 1`. Patched to drop the `system` Boost component, since Boost.System is header-only from Boost 1.89 (26.04 ships 1.90). |
| `velodyne` | Not released | Only an `exec_depend` of `rtabmap_examples`, which this project doesn't use. Skipped in rosdep. |
| `sparkcan` | PPA has no resolute build | Built from source (github.com/grayson-arendt/sparkcan). Its CMake config doesn't find Threads, so `find_package(Threads)` comes first. |
| `livox_laser_simulation_RO2` | Gazebo Classic only | Replaced by `gpu_lidar` + `pointcloud_to_livox` (see above) |
| `libgl1-mesa-glx` | Removed from Ubuntu | `libgl1` |

### Docker / scripts / docs
- Dockerfile: Lyrical base and package names. sparkcan and libpointmatcher are built from source. The `grid_map` removal step is gone.
  The image's built-in `ubuntu` user (uid 1000 since 24.04) is deleted so `lunabot` can take the host uid. No more
  sourcing of `/usr/share/gazebo/setup.bash`.
- `run.sh`: image, container and volume names changed from `humble` to `lyrical`.
- `install_dependencies.sh` / `setup_robot_services.sh`: support `resolute` → `lyrical` only.
- README: Lyrical install and run steps.

## Decisions made without asking
- Hosts on Jammy and Noble are no longer supported on this branch. Gazebo Jetty with gz_ros2_control only exists for Lyrical, and keeping
  several distros working at once would mean maintaining two simulators.
- `/cmd_vel` stays unstamped across the stack, and only the sim controller input is stamped. That kept the change to one node instead of five.
- The simulated lidar resolution (360 × 32) matches the old plugin's published point count (~10k points per scan).

## Verification

Verification ran natively on an Ubuntu 26.04 host with ROS 2 Lyrical and Gazebo Jetty (gz-sim 10), not in Docker. Docker
and sudo weren't available, so the missing ROS debs were extracted into a user-local prefix.

| Check | Result |
|---|---|
| `colcon build` (all 28 packages, incl. rtabmap 0.23.13 with libpointmatcher and `RTABMAP_SYNC_MULTI_RGBD=ON`) | ✅ |
| `colcon test` (`lunabot_*`, `fast_lio`, `point_lio`, `livox_ros_driver2`) | ✅ 1 gtest (`test_pointcloud_to_livox`). The other packages have no tests. |
| All 8 worlds: `gz sdf -k` | ✅ Valid |
| Both robots: xacro → `gz sdf -p` | ✅ 3 rgbd_camera, 1 gpu_lidar, 1 imu, gz_ros2_control plugin |
| Headless sim (`gui_launch.py use_sim:=true sim_gui:=false`, v2_bot, artemis) | ✅ robot spawns |
| Controllers | ✅ joint_state_broadcaster, diff_drive_controller, camera_controller, position_controller all `active` |
| Topics | ✅ `/clock`, `/livox/imu`, `/livox/lidar_points`, `/livox/lidar` (~11k points per scan), all 3 cameras' color and depth, `/joint_states`, `/wheel_odom`, `/tf`, with correct `frame_id`s |
| Drive: `/cmd_vel` → `/wheel_odom` | ✅ x went from 0.0 to 0.39 m |
| Same sim with the Gazebo GUI | ✅ launched and all controllers active (run by the user) |

### Not verified
- **Docker image build**: Docker isn't installed on the verification host. The Dockerfile follows the same steps that
  were verified natively.
- **`install_dependencies.sh`**: needs sudo, so it wasn't run. Its source builds match what was built locally.
- **LIO / RTAB-Map / Nav2 in the loop**: the packages build and their inputs (`/livox/lidar`, `/livox/imu`, cameras)
  publish, but `pointlio_launch.py`, `mapping_launch.py` and `nav2_stack_launch.py` were not run against the sim.
  The Nav2 params and behavior trees haven't been checked against Nav2 Lyrical parameter changes.
- **Real hardware** (`hardware_launch.py`, CAN, Livox driver, RealSense).
- The sim ran well below real time on the test laptop, which renders all four sensors in software, so sensor rates there are not representative.
- The bucket position command was sent, but the resulting joint motion wasn't checked.
