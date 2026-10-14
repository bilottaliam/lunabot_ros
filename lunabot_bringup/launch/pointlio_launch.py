import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import OpaqueFunction


def launch_setup(context, *args, **kwargs):
    config_dir = get_package_share_directory("lunabot_config")
    use_sim = LaunchConfiguration("use_sim").perform(context)
    test_odom = LaunchConfiguration("test_odom").perform(context)   # TEST ONLY, sim: physics odometry
    
    if use_sim.lower() == "true":
        point_lio_config = os.path.join(
            config_dir, "params", "point_lio", "mid360_sim.yaml"
        )
    else:
        point_lio_config = os.path.join(
            config_dir, "params", "point_lio", "mid360_real.yaml"
        )

    ukf_params_file = os.path.join(
        config_dir, "params", "robot_localization", "ukf_params.yaml"
    )

    livox_reorient_node = Node(
        package="lunabot_util",
        executable="livox_reorient",
        name="livox_reorient",
        output="screen",
        parameters=[
            {
                "use_sim_time": LaunchConfiguration("use_sim"),
                "target_frame": "base_link",
                "lidar_frame": "livox_frame",
                "imu_frame": "mid360_imu_link",
                "body_min": [-0.55, -0.50, -0.10],
                "body_max": [0.70, 0.50, 0.80],
                "input_lidar_topic": "/livox/lidar",
                "input_imu_topic": "/livox/imu",
                "output_lidar_topic": "/livox/lidar_body",
                "output_imu_topic": "/livox/imu_body",
            }
        ],
    )

    point_lio_node = Node(
        package="point_lio",
        executable="pointlio_mapping",
        name="laserMapping",
        output="screen",
        parameters=[
            point_lio_config,
            {
                "use_sim_time": LaunchConfiguration("use_sim"),
            },
        ],
        remappings=[
            ("/aft_mapped_to_init", "/lio_odom"),
        ],
    )

    ukf_node = Node(
        package="robot_localization",
        executable="ukf_node",
        name="ukf_filter_node",
        output="screen",
        parameters=[
            ukf_params_file,
            {"use_sim_time": LaunchConfiguration("use_sim")},
        ],
    )

    # Sim only: fuse Point-LIO with wheel odometry + gyro (ekf_sim.yaml). The real robot has no wheel
    # odometry topic, so it keeps Point-LIO's own odom->base_link.
    ekf_sim_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[os.path.join(config_dir, "params", "robot_localization",
                                 "ekf_sim_test_odom.yaml" if test_odom == "true" else "ekf_sim.yaml"),
                    {"use_sim_time": True}],
    )
    return [livox_reorient_node, point_lio_node] + ([ekf_sim_node] if use_sim.lower() == "true" else [])


def generate_launch_description():
    declare_use_sim = DeclareLaunchArgument(
        "use_sim",
        default_value="false",
        description="Whether we are in simulation or not",
    )

    return LaunchDescription([
        declare_use_sim,
        DeclareLaunchArgument("test_odom", default_value="false", choices=["true", "false"]),
        OpaqueFunction(function=launch_setup),
    ])
