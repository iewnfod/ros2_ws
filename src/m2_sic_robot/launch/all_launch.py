import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource


def generate_launch_description():
    hardware_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("m2_sic_robot"), "launch"),
                "/hardware_launch.py",
            ]
        )
    )
    fsm_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("m2_sic_robot"), "launch"),
                "/fsm_launch.py",
            ]
        )
    )
    joy_node = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("m2_joy"), "launch"),
                "/joy_msg.launch.xml"
            ]
        )
    )
    chassis_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("m2_sic_robot"), "launch"),
                "/chassis_launch.py"
            ]
        )
    )

    return LaunchDescription(
        [
            joy_node,
            hardware_nodes,
            # fsm_node,
            chassis_node
        ]
    )
