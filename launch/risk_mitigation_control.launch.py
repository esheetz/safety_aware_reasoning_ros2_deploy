from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # DECLARE ARGUMENTS
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'robot',
            default_value='clr',
            description='Name of robot.',
            choices=['val', 'clr']
        )
    )

    # DECLARE LAUNCH CONFIGURATIONS
    robot = LaunchConfiguration('robot')

    # RISK MITIGATION CONTROL NODE
    risk_mitigation = Node(
        package='safety_aware_reasoning_ros2_deploy',
        executable='risk_mitigation_control_node.py',
        name='SAR_Risk_Mitigation_Control',
        parameters=[{
            "robot" : robot
        }]
    )

    nodes = [risk_mitigation]

    # LAUNCH DESCRIPTION
    return LaunchDescription(declared_arguments + nodes)
