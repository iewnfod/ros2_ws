from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="m2_sic_robot",
                executable="chassis_node",
                name="chassis_node",
                output="screen",
                parameters=[
                    {
                    }
                ]
            )
        ]
    )