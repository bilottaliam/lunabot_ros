"""Headless full-stack sim run on a generated 2027 arena: sim -> Point-LIO -> RTAB-Map -> Nav2 -> autonomy.

    ros2 launch lunabot_bringup sim_autonomy_launch.py seed:=3 random_start:=true start_mission:=true

Stages start on wall-clock delays so each finds its inputs (the sim runs below real time on a laptop).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def include(name, delay, **args):
    src = PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("lunabot_bringup"), "launch", name))
    inc = IncludeLaunchDescription(src, launch_arguments=args.items())
    return TimerAction(period=float(delay), actions=[inc]) if delay else inc


def generate_launch_description():
    lc = LaunchConfiguration
    return LaunchDescription([
        DeclareLaunchArgument("seed", default_value="0"),
        DeclareLaunchArgument("random_start", default_value="true"),
        DeclareLaunchArgument("robot_heading", default_value="east"),
        DeclareLaunchArgument("sim_gui", default_value="false"),
        DeclareLaunchArgument("start_mission", default_value="true"),
        DeclareLaunchArgument("planner_id", default_value="HybridRS"),
        DeclareLaunchArgument("max_cycles", default_value="0"),
        DeclareLaunchArgument("run_duration", default_value="900.0"),
        include("gui_launch.py", 0, use_sim="true", arena_type="artemis2027", seed=lc("seed"),
                random_start=lc("random_start"), robot_heading=lc("robot_heading"), sim_gui=lc("sim_gui"),
                viz_mode="none"),
        include("pointlio_launch.py", 20, use_sim="true"),
        include("mapping_launch.py", 30, use_sim="true"),
        include("nav2_stack_launch.py", 40, use_sim="true"),
        include("autonomy_launch.py", 45, use_sim="true", start_mission=lc("start_mission"),
                planner_id=lc("planner_id"), max_cycles=lc("max_cycles"), run_duration=lc("run_duration")),
    ])
