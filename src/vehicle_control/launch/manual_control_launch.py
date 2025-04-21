import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import subprocess


def generate_launch_description():    
    ld = LaunchDescription()

    vesc_config = os.path.join(
        get_package_share_directory('vehicle_control'),
        'config',
        'vesc_config.yaml'
        )

    control_config = os.path.join(
        get_package_share_directory('vehicle_control'),
        'config',
        'control_config.yaml'
        )
    
    vesc=Node(
        package = 'vesc_driver',
        name = 'vesc_driver_node',
        executable = 'vesc_driver_node',
        parameters = [vesc_config],
        respawn=True,
        respawn_delay=20.0
    )

    micro_ros_agent = Node(
        package='micro_ros_agent',
        name="micro_ros_agent",
        executable="micro_ros_agent",
        arguments=["serial", "-b", "921600", "--dev", "/dev/stm32_nucleo"],
        respawn=True,
        respawn_delay=20.0
    )
    
    rc2joy=Node(
        package = 'vehicle_control',
        name = 'rc_to_joy',
        executable = 'rc_to_joy',
        parameters = [control_config]
    )

    joy2ackermann=Node(
        package = 'vehicle_control',
        name = 'joy_to_ackermann',
        executable = 'joy_to_ackermann',
        parameters = [control_config]
    )

    ackermann2vesc=Node(
        package = 'vehicle_control',
        name = 'ackermann_to_vesc',
        executable = 'ackermann_to_vesc',
        parameters = [control_config],
        output="screen"
    )

    # ===== Launch missing ROS2 nodes =====

    # Get list of currently running nodes
    result = subprocess.run(['ros2', 'node', 'list'], stdout=subprocess.PIPE, text=True)
    
    # Get node names and remove the leading slash
    running_nodes = [node.lstrip('/') for node in result.stdout.strip().split('\n')] if result.stdout else []

    # Map node name strings to their executables
    node_names = {
        'rc_to_joy': rc2joy,
        'joy_to_ackermann': joy2ackermann,
        'ackermann_to_vesc': ackermann2vesc,
        'vesc_driver_node': vesc,
        'micro_ros_agent': micro_ros_agent
    }

    # Launch node if it's not already running
    for name, node in node_names.items():
        if not name in running_nodes:
            ld.add_action(node)

    
    return ld



