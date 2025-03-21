from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable
from launch_ros.parameter_descriptions import ParameterValue
import os
from pathlib import Path
from ament_index_python.packages import get_package_prefix

def generate_launch_description():

    # Path to the URDF file in the source directory
    pkg_dir = get_package_prefix('mxck_run').replace('install', 'src') #  /mxck2_ws/install/mxck_run → /humble_ws/src/mxck_run
    urdf_path = pkg_dir + '/urdf/mxcarkit.urdf'

    
    with open(urdf_path, 'r') as urdf_file:
        robot_description_content = urdf_file.read()

    robot_description = ParameterValue(robot_description_content, value_type=str)

    # Define nodes
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'robot_description': robot_description},
            {'publish_frequency': 20.0},  # Adjust this value as needed
            {'use_tf_static': True}  # Set to True to publish static transforms only once
        ]
    )

    return LaunchDescription([
        robot_state_publisher_node
    ])