from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import subprocess

def generate_launch_description():

    ld = LaunchDescription()

    # Path to the URDF file in the source directory
    urdf_path = get_package_share_directory('mxck_run') + '/urdf/mxcarkit.urdf' # /mxck2_ws/install/mxck_run/share/mxck_run

    with open(urdf_path, 'r') as urdf_file:
        robot_description_content = urdf_file.read()

    robot_description = ParameterValue(robot_description_content, value_type=str)

    # Define nodes
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name = 'robot_state_publisher',
        parameters=[
            {'robot_description': robot_description},
            {'publish_frequency': 20.0},  # Adjust this value as needed
            {'use_tf_static': True}  # True: publish /tf_static once; False: publish /tf_static at publish_frequency.
        ]
    )

    # ===== Launch missing ROS2 nodes =====

    # Get list of currently running nodes
    result = subprocess.run(['ros2', 'node', 'list'], stdout=subprocess.PIPE, text=True)
    
    # Get node names and remove the leading slash
    running_nodes = [node.lstrip('/') for node in result.stdout.strip().split('\n')] if result.stdout else []

    # Map node name strings to their executables
    node_names = {
        'robot_state_publisher': robot_state_publisher,
    }

    # Launch node if it's not already running
    for name, node in node_names.items():
        if not name in running_nodes:
            ld.add_action(node)

    return ld