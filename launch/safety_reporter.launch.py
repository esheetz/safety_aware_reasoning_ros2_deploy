from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # SAFETY REPORTER NODE
    reporter = Node(
        package='safety_aware_reasoning_ros2_deploy',
        executable='safety_reporter_node.py',
        name='SAR_Safety_Reporter'
    )

    # LAUNCH DESCRIPTION
    return LaunchDescription([reporter])
