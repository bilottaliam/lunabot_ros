# Review of the Lyrical upgrade (`upgrade/lyrical` vs `master`)

An audit of the upgrade described in [MIGRATION.md](MIGRATION.md). The goals: the real robot boots and runs as it does today,
the sim matches the real robot, and the slow rotation is found and fixed.

## Summary

- **Real robot path is unchanged apart from build/API changes.** `hardware_launch.py`, `actions_launch.py`, the
  LIO/mapping/Nav2 launch files, the teleop and motor code, the Nav2/LIO/RTAB-Map/UKF params, serial ports, baud rates, the
  CAN and udev setup, and the Livox config have no diff against `master`. The C++ changes are `tf2/*.h` → `.hpp` includes
  and CMake linking.
- **Real-robot blockers that the upgrade introduced, now fixed:**
  - `robot_upstart` has no Lyrical release, so `setup_robot_services.sh` couldn't install the boot service. It is now vendored.
  - The Dockerfile and `install_dependencies.sh` failed at the Livox-SDK2 build under CMake 4. They also built sparkcan
    (the motor library) from an unpinned upstream branch, and Docker couldn't see the new install script. All three are fixed.
- **Rotation:** turning in place reached **40%** of the commanded yaw rate (straight driving: 68%). It now reaches
  **99–103%**, straight driving **100%**, and wheel odometry agrees with ground truth. The root cause is skid-steer wheel scrub;
  see [Rotation](#rotation).
- **Sim vs real:** the sim cameras rendered 4–12× the real frame rates at the wrong aspect ratio, and two topic names had
  drifted from `master`. Both are fixed. The sim now runs at about 0.7× real time on the test laptop, up from about 0.1–0.2×.
- **Not tested on hardware:** the robot itself. See [Still needs testing on the real robot](#still-needs-testing-on-the-real-robot).

## Rotation

### Root cause

The robot is a **4-wheel skid steer** (wheelbase 0.45 m, track 0.50 m). `diff_drive_controller` models a no-slip
differential drive, but a skid steer can only turn by dragging its wheels sideways. For the same wheel speeds the robot turns
slower than the kinematic model predicts, while wheel odometry still reports the commanded rate. A Coulomb-friction
model of this geometry predicts about 55% of the commanded yaw; the sim measured 61%.

`master` also had `wheel_radius: 0.15` against 0.1016 m wheels in the URDF. That alone made everything 68% of commanded,
so rotation landed at 0.68 × 61% ≈ 40%.

### Compared old vs new

| Item | `master` | Now | Note |
|---|---|---|---|
| wheel_radius (sim controller) | 0.15 | **0.1016** | Matches the URDF wheel (4") |
| wheel_separation | 0.5 | 0.5 | Matches URDF wheel y = ±0.25 |
| wheel_separation_multiplier | (1.0) | **1.64** | Effective skid-steer track 0.82 m (the fix) |
| Wheel velocity command limit | ±1 rad/s, never enforced | ±15 rad/s, enforced | Lyrical enforces limits; ±1 would cap the robot at 0.10 m/s |
| Angular velocity/accel limits in diff_drive | none | none | Unchanged |
| Nav2 velocity smoother / RPP params | — | unchanged | `nav2_*_params.yaml` have no diff |
| `/cmd_vel` scaling | none | none | `topic_remapper` only adds a stamp |
| Wheel/ground friction | defaults (`<mu>` was silently ignored) | `mu1/mu2` 1.0, `kp/kd` | Ground mu 0.4 governs in both |
| Masses, inertias, joint geometry | — | identical | Diffed with `master` |

### Measurement

Gazebo ground truth (`/world/default/pose/info`) vs command. Before each trial the robot is teleported back to the artemis
spawn pose. Script: `build_env/tools/motion_test.py` (outside the repo).

| Trial | Before (`master` controller config) | After |
|---|---|---|
| Spin 0.5 rad/s | 0.201 rad/s (**40%**), odom says 0.500 | 0.514 rad/s (**103%**), odom 0.500 |
| Spin 1.0 rad/s | 0.404 rad/s (**40%**), odom says 1.000 | 0.987 rad/s (**99%**), odom 1.000 |
| Straight 0.35 m/s | 0.238 m/s (**68%**), odom says 0.350 | 0.350 m/s (**100%**), odom 0.350 |
| Coast after stop | 2.4 cm | 4.9 cm (real 0.35 m/s, ∝ v²) |

The multiplier fixes the cause in the model, not a limit: it corrects both the wheel speeds diff_drive commands and the
odometry it reports. The ±15 rad/s wheel limit is sized so the GUI teleop maximum (0.75 m/s + 0.75 rad/s → about 10.4 rad/s)
isn't clipped.

### Real robot

The real robot doesn't use `diff_drive_controller`.
- **Joystick:** wheel duty = `linear ± rotation`, with no kinematic model.
- **Autonomous:** `controller_teleop` (`velocity_callback`) converts `/cmd_vel` with `wheel_base = 0.53`, `wheel_radius = 0.095`.

Both are unchanged and open loop. By the same skid-steer physics, the real robot also turns slower than a commanded
`angular.z`. Nav2 closes the loop through LIO odometry, so it still converges, just with less turning authority than its
params assume. I did **not** change the real-robot code without hardware to verify against. Calibrate it on the robot:
1. Command `angular.z = 0.5` in autonomous mode.
2. Measure the yaw rate from `/lio_odom`.
3. Set an effective `wheel_base` of 0.53 × (0.5 / measured) in `controller_teleop.cpp`.

## Sim accuracy

| Check | Result |
|---|---|
| Masses, inertias, joint origins/axes/limits, wheel cylinders | Identical to `master` (diffed per tag) |
| Sensor placement (joint origins) | Identical |
| World physics block (step 0.001 s, RTF 1, 1000 Hz, gravity) | Identical |
| Physics engine | **Differs, unavoidably**: gz-sim ignores `<physics type='ode'>` and uses DART (with ODE collision detection); Classic used ODE |
| Wheel friction | Old `<mu>`/`<surface><ode>` (incl. `slip1/2`) were never mapped by sdformat, so Classic ran on defaults. Now `mu1/mu2 1.0`, `kp 1e6`, `kd 500` actually apply. Ground mu 0.4 is the limiting value in both. |
| D456 cameras | Now 640×480 @ 15 Hz like `hardware_launch.py` (old sim: 640×360 @ 60 Hz) |
| Fisheye camera | Now 480×360 @ 5 Hz like `hardware_launch.py` (old sim: 640×360 @ 60 Hz) |
| IMU | 100 Hz, noiseless, as in old sim (the Classic ROS 2 plugin ignored `<gaussianNoise>`) |
| Lidar | 10 Hz, same vertical FOV and range; regular 360×32 grid instead of the Livox rosette (no gz-sim Livox plugin exists) |
| Ground truth vs commands | Straight, turning and stopping measured above |

Gazebo Classic doesn't exist on 26.04, so the "old" column is `master`'s controller config run on gz-sim, plus code
review of what Classic did. I could not run the Classic simulator side by side.

## Diff against `master`

Excluding the verbatim rtabmap/rtabmap_ros re-vendor and the deleted Classic Livox plugin.

| Change | Class | Disposition |
|---|---|---|
| Ubuntu/ROS/Gazebo versions in Dockerfile, scripts, README | Required | Kept |
| `ament_target_dependencies` → `target_link_libraries`, `tf2/*.h` → `.hpp`, `find_package(ament_cmake/Threads)` | Required (API removed) | Kept; duplicate `find_package(Threads)` removed |
| rtabmap 0.21.9 → 0.23.13-lyrical | Required (old doesn't build) | Kept; old copy verified unmodified upstream |
| robot_upstart vendored | Required (no release) | Added in review |
| Source deps (sparkcan, Livox SDK2, libnabo, libpointmatcher) | Required (no 26.04 packages) | One pinned script, used by Docker and native installs |
| Gazebo Classic → gz-sim (URDF sensors, worlds, launch, bridge, env hook) | Required | Kept |
| `pointcloud_to_livox` node + test | Required (replaces Classic-only plugin) | Kept |
| `topic_remapper` publishes TwistStamped | Required (diff_drive API) | Kept; runs on sim time |
| `enable_stamped_cmd_vel: false` for Nav2 | Required to keep `/cmd_vel` a Twist for the real robot | Kept |
| Controller types → ForwardCommandController | Required (old type removed) | Kept; same topics/messages |
| Spawner `--param-file`, single spawner, long timeouts | Required (params no longer inherited; slow sims timed out) | Kept |
| Command limits ±15 rad/s / bucket [-1.57, 0.1] | Behavior-changing (limits now enforced) | Justified: Humble ignored them; the values allow the old range of motion |
| `wheel_radius` 0.15 → 0.1016, multiplier 1.64 | Behavior-changing (sim) | Justified: fixes a `master` bug; the sim now drives as commanded |
| Sim camera profiles | Behavior-changing (sim) | Justified: matches the real robot |
| `QT_QPA_PLATFORM=xcb` for the GUI | Required on 26.04 (no keyboard input on native Wayland) | Kept; affects the operator laptop in both modes |
| `robot_description` as `ParameterValue(str)` | Required (launch failed on a `": "` in a comment) | Kept; same value as before |
| `lunabot_gui` exec_depends pyqt5/opencv | Not strictly required | Kept: lets rosdep install what the script installed by hand |
| livox_ros_driver2 linter excludes | Test-only | Kept: vendored code, no functional tests |
| Classic `gazebo_ros` export in lunabot_config | Cleanup | Removed (nothing reads it) |

No other unnecessary changes were found. Each commit on the branch is one logical change.

## Interface check (names that must not change)

Checked mechanically (launch arguments, node packages/executables/names in every launch file) and by review:

- **Unchanged:** all launch arguments; all real-robot nodes, topics, services, actions and parameters; `lunabot_msgs`
  (msg/srv/action); TF frames; message types on every topic the robot or GUI uses (`/cmd_vel` stays `geometry_msgs/Twist`).
- **Sim-only differences:**
  - `spawn_entity.py` → `ros_gz_sim create`; four spawners → one; new `parameter_bridge` and `pointcloud_to_livox` nodes.
  - The internal topic `/diff_drive_controller/cmd_vel_unstamped` (Twist) became `/diff_drive_controller/cmd_vel`
    (TwistStamped). Only `topic_remapper` uses it.
  - Sim `/livox/lidar` `frame_id` is now `livox_frame`, the same as the real driver. The old sim used `mid360_lidar_sensor`;
    `livox_reorient` overrides it anyway.
  - `/livox/lidar_PointCloud2` and `/camera_*/depth/camera_info` keep their `master` names.
  - Gazebo-internal (gz transport) topic names are new; they aren't ROS interfaces.

## Real robot compatibility

| Dependency | Status |
|---|---|
| CAN (`canable_start.sh`: can0 @ 1 Mbit/s, `canable-setup.service`) | Unchanged |
| Serial: power monitor `/dev/ttyACM0` @ 9600, fisheye pan `/dev/ttyACM1` @ 115200 | Unchanged (launch params, no diff) |
| USB: D456 ×2 by serial number, `/dev/webcam_fisheye`, RealSense udev rules | Unchanged |
| sparkcan (SparkMax CAN) | No PPA build for 26.04 → source build pinned to `11d2d13` (the current upstream HEAD) |
| Livox SDK2 + livox_ros_driver2 | Source build pinned; driver builds on Lyrical; config unchanged |
| realsense2_camera | `ros-lyrical-realsense2-camera` 4.58.4. All 15 launch arguments used by `hardware_launch.py` still exist. |
| joy, robot_localization, imu_complementary_filter, laser_filters, foxglove_bridge, Nav2, image transport plugins, pcl_ros, cv_bridge | Released for Lyrical |
| python3-serial, python3-opencv, python3-pyqt5, can-utils | In 26.04 |
| robot_upstart (boot service) | Not released → vendored. Generated unit and start/stop scripts verified with a temporary root. |
| Microcontroller protocols | Unchanged (no firmware or protocol code in the diff) |
| DDS / domain | Default RMW is still `rmw_fastrtps_cpp`; the boot service uses ROS_DOMAIN_ID 0, as before |
| Docker | Sim-only, as before. `--net=host`, `/dev/dri`, video/render/dialout groups unchanged; CAN/serial stay on the host |

### Boot checklist (real robot, Ubuntu 26.04)

1. Install ROS 2 Lyrical: `ros-lyrical-desktop` and `ros-dev-tools`.
2. Clone the repo:
   ```bash
   mkdir -p ~/lunabot_ws/src && cd ~/lunabot_ws/src && git clone <repo> && cd lunabot_ros && git checkout upgrade/lyrical
   ```
3. Install dependencies. This also builds sparkcan, Livox SDK2 and libpointmatcher:
   ```bash
   scripts/install_dependencies.sh
   ```
4. Install the udev rules:
   ```bash
   sudo scripts/setup_udev_rules.sh
   ```
5. Build the workspace:
   ```bash
   cd ~/lunabot_ws && MAKEFLAGS=-j1 colcon build --symlink-install --cmake-args -DRTABMAP_SYNC_MULTI_RGBD=ON -DWITH_OPENCV=ON -DWITH_APRILTAG=ON -DWITH_OPENGV=OFF -DCMAKE_POLICY_VERSION_MINIMUM=3.5 --parallel-workers 1
   ```
6. Install the CAN and launch-manager boot services:
   ```bash
   sudo src/lunabot_ros/scripts/setup_robot_services.sh
   ```
   Then reboot, or start both services.
7. Check that CAN is up (`ip link show can0` should report UP) and that the launch manager is running
   (`journalctl -u lunabot-launch-manager`).
8. From the operator laptop, run `ros2 launch lunabot_bringup gui_launch.py`, then press Launch Hardware. Check:
   - `/livox/lidar` at 10 Hz
   - `/camera_front/color/image_raw` and `/camera_back/color/image_raw` at 15 Hz
   - `/camera_fisheye/...` images
   - the power monitor values
   - each motor's response to the joystick: wheels, bucket, fisheye pan
9. Run Point-LIO, then RTAB-Map, then Nav2 from the GUI and do a short autonomous move.

No microcontroller flashing is needed: no firmware or serial protocol changed.

## Self-review

An independent subagent reviewed the diff read-only. It confirmed the real-robot path is build/API-only and that no
interface names were dropped. Its findings:

| # | Finding | Action |
|---|---|---|
| 1 | Duplicate `find_package(Threads)` | Fixed |
| 2 | sparkcan, libnabo and libpointmatcher built from unpinned upstream; the Boost `sed` could silently do nothing | Fixed: pinned commits; the patch fails loudly |
| 3 | Livox `<cstdint>` patch dropped | Not needed: upstream builds with GCC 15 (verified) |
| 4 | CMake policy flag only on some builds in Docker | Fixed (shared script) |
| 5 | libpointmatcher re-run bug in the install script | Fixed (shared script, temp dir) |
| 6 | `/livox/lidar_PointCloud2` and `depth/camera_info` dropped | Fixed: same names as `master` |
| 7 | Camera `points` cloud orientation | Checked in the sim; see [Verification](#verification) |
| 8 | Lidar duplicated its first column | Fixed (`max_angle` π − 1°) |
| 9 | IMU noise added (old sim was noiseless) | Fixed: noiseless again |
| 10 | Spawner comment overstated batching | Fixed |
| 11 | wheel_radius changes the sim drive speed | Kept, documented (bug fix; now measured) |
| 12 | xcb also applies in real mode on the laptop | Kept, documented |
| 13 | Stale docs | Fixed |
| 14 | gpu_lidar vs semi-transparent walls in the low-res worlds | Not verified (default worlds have no transparency) |

Found while rebuilding from scratch and testing:
- The Livox-SDK2 CMake 4 failure.
- `.dockerignore` hiding the install script from Docker.
- The robot_upstart blocker.
- The `robot_description` YAML-parse failure.
- The camera-rate mismatch.

## Verification

Native Ubuntu 26.04 + Lyrical host. Docker and sudo aren't available here, so the missing Lyrical packages were extracted
into a user-local overlay. The build environment lives outside the repo, in `build_env/` next to it.

| Check | Result |
|---|---|
| Clean rebuild of all 30 packages (incl. rtabmap with libpointmatcher and `RTABMAP_SYNC_MULTI_RGBD=ON`) | ✅ |
| Source deps via the same steps as `install_source_deps.sh` (sparkcan, Livox SDK2, libnabo, libpointmatcher) | ✅ (found and fixed the CMake 4 failure) |
| rosdep on the whole repo (`--skip-keys "libpointmatcher velodyne"`) | ✅ every key resolves |
| `colcon test` | ✅ `test_pointcloud_to_livox` passes. The other packages have no functional tests. robot_upstart's upstream tests pass with pytest (5/5); colcon here fell back to unittest and found none. |
| robot_upstart generates the boot service | ✅ unit and start/stop scripts generated into a temporary root |
| Headless sim | ✅ spawns, 4/4 controllers active, about 0.7× real time |
| Sensors (by sim timestamps) | ✅ lidar 10 Hz, IMU 100 Hz, front camera_info 15 Hz, fisheye 5 Hz, no duplicate messages; frames match the URDF |
| Drive and rotation vs ground truth | ✅ see [Rotation](#rotation) |
| GUI teleop | ✅ earlier session: WASD → `/cmd_vel` (QTest and by hand) after the xcb fix |
| Point-LIO in sim | ✅ IMU init completes, `/lio_odom` publishes |
| RTAB-Map in sim | ✅ publishes `map → odom` |
| **Nav2 in sim** | ⚠️ **In progress.** Nav2 Lyrical config changes in `nav2_v1_bot_params.yaml` (sim only):<br>• Fixed: MPPI motion model is a plugin (`DiffDrive: plugin: "mppi::DiffDriveMotionModel"`).<br>• Fixed: `pkg/Class` plugin names → `pkg::Class` (bt_navigator, smac planner, behaviors).<br>• Next: bt_navigator "`error_code_names` has been replaced by `error_code_name_prefixes`". controller/planner/behavior servers now configure. Check script: `build_env/tools/nav2_config_check.sh true\|false`. |
| Docker image build | ⚠️ not possible here (no Docker) |

Two problems in my own test tooling were found and fixed during this review; neither is in the repo:
- Killing launches with `kill -9` left orphaned sim nodes, which duplicated messages and slowed the sim. Launches now run in systemd scopes.
- The build was in `/tmp`, a RAM disk on 26.04 that's wiped on reboot. It's now on disk.

## Still needs testing on the real robot

1. **Nav2 params on Lyrical.** The sim params fail as above. `nav2_real_bot_params.yaml` uses other plugins and hasn't been
   run on Lyrical; check that every Nav2 node reaches `active` before relying on autonomy.
2. Full boot checklist above: CAN up, launch-manager service, sensors at the expected rates, motors respond.
3. sparkcan at commit `11d2d13` behaves like the PPA build the robot used before. Drive each motor at low duty first.
4. Real-robot turning rate in autonomous mode (calibrate `wheel_base` in `controller_teleop.cpp` as described in
   [Rotation](#rotation)).
5. RealSense driver 4.58 on the D456s (parameter names verified, hardware not).
6. Point-LIO/RTAB-Map on real data with the Lyrical builds.
7. Docker image build on a machine with Docker.
8. gpu_lidar returns from the semi-transparent walls in the low-resolution worlds.
