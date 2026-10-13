"""Autonomy layer on top of Point-LIO, RTAB-Map and Nav2: arena frame from tags, terrain obstacles,
and (with start_mission:=true) the mission executive.

    ros2 launch lunabot_bringup autonomy_launch.py use_sim:=true start_mission:=true
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config_dir = get_package_share_directory("lunabot_config")
    use_sim = LaunchConfiguration("use_sim")
    sim_time = {"use_sim_time": PythonExpression(["'", use_sim, "' == 'true'"])}

    return LaunchDescription([
        DeclareLaunchArgument("use_sim", default_value="false"),
        DeclareLaunchArgument("start_mission", default_value="false",
                              description="Start the mission executive (hands-free run) immediately."),
        DeclareLaunchArgument("tags_file", default_value=os.path.join(config_dir, "params", "arena", "arena_tags.yaml")),
        DeclareLaunchArgument("planner_id", default_value="HybridRS"),
        DeclareLaunchArgument("max_cycles", default_value="0"),
        DeclareLaunchArgument("run_duration", default_value="900.0"),
        Node(
            package="apriltag_ros", executable="apriltag_node", name="apriltag",
            parameters=[os.path.join(config_dir, "params", "apriltag", "fisheye_36h11.yaml"), sim_time],
            remappings=[("image_rect", "/camera_fisheye/color/image_raw"),
                        ("camera_info", "/camera_fisheye/color/camera_info")],
        ),
        Node(
            package="lunabot_nav", executable="arena_localizer.py", name="arena_localizer", output="screen",
            parameters=[{"tags_file": LaunchConfiguration("tags_file")}, sim_time],
        ),
        Node(
            package="lunabot_nav", executable="terrain_mapper.py", name="terrain_mapper", output="screen",
            parameters=[sim_time],
        ),
        Node(
            package="lunabot_nav", executable="mission_executive.py", name="mission_executive", output="screen",
            parameters=[sim_time, {"planner_id": ParameterValue(LaunchConfiguration("planner_id"), value_type=str),
                                   "max_cycles": ParameterValue(LaunchConfiguration("max_cycles"), value_type=int),
                                   "run_duration": ParameterValue(LaunchConfiguration("run_duration"), value_type=float)}],
            condition=IfCondition(LaunchConfiguration("start_mission")),
        ),
    ])
