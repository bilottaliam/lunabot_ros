"""Robot side of the MCC link: foxglove_bridge (allow-listed) + mcc_gateway (arena-frame summaries).

Start it with the launch manager at boot so the MCC can reach the robot before anything else runs.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(get_package_share_directory("lunabot_config"), "params", "mcc", "foxglove_bridge.yaml")
    sim_time = {"use_sim_time": LaunchConfiguration("use_sim")}
    return LaunchDescription([
        DeclareLaunchArgument("use_sim", default_value="false"),
        Node(package="foxglove_bridge", executable="foxglove_bridge", name="foxglove_bridge",
             parameters=[params, sim_time]),
        Node(package="lunabot_util", executable="mcc_gateway.py", name="mcc_gateway", parameters=[sim_time]),
    ])
