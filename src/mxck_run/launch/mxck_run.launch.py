from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare 
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Declare the 'run_foxglove' argument with a default value of 'false'
        DeclareLaunchArgument(
            'run_foxglove',
            default_value='false',
            description='Flag to run foxglove bridge'
        ),

        # Declare the 'run_camera' argument with a default value of 'false'
        DeclareLaunchArgument(
            'run_camera',
            default_value='false',
            description='Flag to run camera'
        ),

        # Declare the 'run_lidar' argument with a default value of 'false'
        DeclareLaunchArgument(
            'run_lidar',
            default_value='false',
            description='Flag to run LiDAR'
        ),

        # Conditionally include the foxglove_bridge launch file
        GroupAction(
            condition=IfCondition(LaunchConfiguration('run_foxglove')),
            actions=[
                IncludeLaunchDescription(
                    XMLLaunchDescriptionSource([
                        FindPackageShare('foxglove_bridge'), '/launch/foxglove_bridge_launch.xml'
                    ]),
                    launch_arguments={
                        'port': '8765',
                        'send_buffer_limit': '10000000',
                        'topic_whitelist': "['/imu_filtered', '/imu_calibrated', '/pdc_visualization', '/camera/color/image_raw', '/bboxs', '/scan']"
                    }.items()
                )
            ]
        ),

        # Conditionally include the realsense2_camera launch file
        GroupAction(
            condition=IfCondition(LaunchConfiguration('run_camera')),
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        FindPackageShare('realsense2_camera'), '/launch/rs_launch.py'
                    ]),
                    launch_arguments={
                        'color_width': '320',
                        'color_height': '180',
                        'color_fps': '6',
                        'enable_confidence': 'False',
                        'enable_depth': 'False',
                    }.items()
                )
            ]
        ),

        # Conditionally include the rplidar_ros launch file for the RPLIDAR A2M8 model
        GroupAction(
            condition=IfCondition(LaunchConfiguration('run_lidar')),
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        FindPackageShare('rplidar_ros'), '/launch/rplidar_a2m8_launch.py'
                    ])
                )
            ]
        ),
    ])
