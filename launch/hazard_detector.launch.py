from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # DECLARE ARGUMENTS
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'launch_color_blob',
            default_value='False',
            description="Flag to launch color blob detection services."
        )
    )

    # DECLARE LAUNCH CONFIGURATIONS
    launch_color_blob = LaunchConfiguration('launch_color_blob')

    # COLOR BLOB DETECTION
    color_blob = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('color_blob_centroid'),
            'launch',
            'debug.launch.py'
        )),
        condition=IfCondition(launch_color_blob)
    )

    # HAZARD DETECTOR NODE
    hazard_detector = Node(
        package='safety_aware_reasoning_ros2_deploy',
        executable='hazard_detector_node.py',
        name='SAR_Hazard_Detector'
    )

    nodes = [color_blob, hazard_detector]

    # LAUNCH DESCRIPTION
    return LaunchDescription(declared_arguments + nodes)
