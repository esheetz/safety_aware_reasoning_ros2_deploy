from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # DECLARE ARGUMENTS
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_name',
            default_value='val_clr',
            description='Name of robot\
                        (Valkyrie and CLR respond the same, so they are a combined robot).'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'env',
            default_value='lunar_habitat',
            description='Name of the environment.',
            choices=['lunar_habitat','household']
        )
    )

    # DECLARE LAUNCH CONFIGURATIONS
    robot_name = LaunchConfiguration('robot_name')
    env = LaunchConfiguration('env')

    # RISKY CONDITION FILE
    risky_condition_file_path = os.path.join(
        get_package_share_directory('safety_aware_reasoning_ros2_deploy'),
        'config'
    )

    # SAVED MODEL FILES
    saved_model_file_path = os.path.join(
        get_package_share_directory('safety_aware_reasoning_ros2_deploy'),
        'saved_models'
    )

    # RISK MITIGATING ACTION PREDICTOR NODE
    action_predictor = Node(
        package='safety_aware_reasoning_ros2_deploy',
        executable='risk_mitigating_action_predictor_node.py',
        name='SAR_Risk_Mitigating_Action_Predictor',
        parameters=[{
            "robot_name" : robot_name,
            "environment" : env,
            "saved_model_file_path" : saved_model_file_path,
            "risky_condition_file_path" : risky_condition_file_path
        }]
    )

    nodes = [action_predictor]

    # LAUNCH DESCRIPTION
    return LaunchDescription(declared_arguments + nodes)
