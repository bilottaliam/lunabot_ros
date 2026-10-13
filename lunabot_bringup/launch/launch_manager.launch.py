import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Keep all robot DDS traffic on the robot; the MCC reaches it only through foxglove_bridge.
        # Inherited by everything the launch manager starts (hardware, LIO, mapping, Nav2, autonomy).
        SetEnvironmentVariable('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST'),
        Node(
            package='lunabot_bringup',
            executable='launch_manager_node.py',
            name='launch_manager',
            output='screen',
            parameters=[],
            emulate_tty=True
        ),
        # The MCC link comes up at boot, so the MCC can reach the launch manager over it
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('lunabot_bringup'), 'launch', 'mcc_link_launch.py'))),
    ])
