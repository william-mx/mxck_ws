import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

    
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
        parameters = [vesc_config]
    )
     
    rc2joy=Node(
        package = 'vehicle_control',
        name = 'rc_to_joy',
        executable = 'rc_to_joy',
        parameters = [vesc_config]
    )

    joy2ackermann=Node(
        package = 'vehicle_control',
        name = 'joy_to_ackermann',
        executable = 'joy_to_ackermann',
        parameters = [vesc_config]
    )

    ackermann2vesc=Node(
        package = 'vehicle_control',
        name = 'ackermann_to_vesc',
        executable = 'ackermann_to_vesc',
        parameters = [vesc_config]
    )
    
    nucleo=Node(
        package = 'micro_ros_agent',
        name = 'micro_ros_agent',
        executable = 'micro_ros_agent',
        arguments=["serial", "-b", "921600", "--dev", "/dev/stm32_nucleo"]
    )

    ld.add_action(vesc)
    ld.add_action(rc2joy)
    ld.add_action(joy2ackermann)
    ld.add_action(nucleo)
    ld.add_action(ackermann2vesc)
    
    return ld
