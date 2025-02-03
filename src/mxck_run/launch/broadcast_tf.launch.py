from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable
from launch_ros.parameter_descriptions import ParameterValue
import os
from pathlib import Path
from ament_index_python.packages import get_package_prefix

def generate_launch_description():
    # Get the package directory (not the install directory)
    pkg_path = get_package_prefix('mxck_run') # /humble_ws/install/mxck_run
    ws_dir = Path(pkg_path).parents[-2] #/humble_ws
    pkg_src_dir = os.path.join(ws_dir, 'src', 'mxck_run')
    
    # Path to the URDF file in the source directory
    urdf_path = os.path.join(pkg_src_dir, 'urdf', 'mxcarkit.urdf')
    
    # Declare the robot_description parameter with proper type handling
    robot_description = ParameterValue(
        Command([
            FindExecutable(name='xacro'),
            ' ',
            urdf_path
        ]),
        value_type=str
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description
        }]
    )

    return LaunchDescription([
        robot_state_publisher_node
    ])