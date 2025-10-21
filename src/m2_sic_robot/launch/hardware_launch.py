from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Define arguments
    ifr_name = LaunchConfiguration("ifr_name", default="can0")
    position_type = LaunchConfiguration("position_type", default="cascaded_pid")

    left_wheel = Node(
        package="m2_vesc",
        executable="vesc_node",
        name="left_wheel",
        output="screen",
        parameters=[
            {"position_type": position_type},
            {"vesc_id": 1},
            {"ifr_name": ifr_name},
        ],
    )

    right_wheel = Node(
        package="m2_vesc",
        executable="vesc_node",
        name="right_wheel",
        output="screen",
        parameters=[
            {"position_type": position_type},
            {"vesc_id": 2},
            {"ifr_name": ifr_name},
        ]
    )

    # Create the launch description
    return LaunchDescription([
        left_wheel,
        right_wheel,
    ])
