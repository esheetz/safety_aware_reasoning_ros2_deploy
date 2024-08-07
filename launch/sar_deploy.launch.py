from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # HAZARD DETECTION
    hazard_detector = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('safety_aware_reasoning_ros2_deploy'),
            'launch',
            'hazard_detector.launch.py'
        ))
    )

    # RISK MITIGATING ACTION PREDICTION
    action_predictor = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('safety_aware_reasoning_ros2_deploy'),
            'launch',
            'risk_mitigating_action_predictor.launch.py'
        ))
    )

    nodes = [hazard_detector, action_predictor]

    # LAUNCH DESCRIPTION
    return LaunchDescription(nodes)
