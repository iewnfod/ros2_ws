from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="m2_sic_robot",
                executable="fsm_node",
                name="fsm_node",
                output="screen",
                parameters=[
                    {
                        "example_set_topic": "/example/setpoint",
                        "example_fb_topic": "/example/feedback",
                        "gripper_mover_set_pos_topic": "gripper_mover/p_setpoint",
                        "gripper_mover_fb_pos_topic": "gripper_mover/p_feedback",
                        "gripper_fingers_grip_topic": "gripper_fingers/grip",
                        "gripper_arms_extend_topic": "gripper_arms/extend",
                        "gripper_mover_min_pos": 0.0,
                        "gripper_mover_max_pos": 600.0,
                        "gripper_rotator_set_pos_topic": "gripper_rotator/p_setpoint",
                        "gripper_rotator_fb_pos_topic": "gripper_rotator/p_feedback",
                        "gripper_rotator_min_pos": 0.0,
                        "gripper_rotator_max_pos": 500.0,
                        "gripper_rotator_right_pos": 0.0,
                        "gripper_rotator_down_pos": 250.0,
                        "gripper_rotator_left_pos": 500.0,
                    }
                ],
            )
        ]
    )
