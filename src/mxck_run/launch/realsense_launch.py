#!/usr/bin/env python3
"""
dynamic_realsense_launch.py

Launch the Intel RealSense camera/IMU/depth streams based on concise flags:
  - camera        : RGB/color camera
  - depth         : depth stream
  - rs_imu        : accel + gyro
  - ir_left       : first IR sensor
  - ir_right      : second IR sensor
  - ir_projector  : IR projector pattern emitter
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

# Define concise launch arguments
SHORT_ARGS = [
    ("camera",       "false", "Enable the RGB/color camera"),
    ("depth",        "false", "Enable the depth stream"),
    ("rs_imu",       "false", "Enable the RealSense IMU (accel + gyro)"),
    ("ir_left",      "false", "Enable the left IR sensor"),
    ("ir_right",     "false", "Enable the right IR sensor"),
    ("ir_projector", "false", "Enable the IR projector pattern emitter"),
]

def generate_launch_description():
    ld = LaunchDescription()

    # Declare all arguments
    for name, default, desc in SHORT_ARGS:
        ld.add_action(
            DeclareLaunchArgument(name, default_value=default, description=desc)
        )

    # Check if any stream is enabled
    any_enabled = PythonExpression([
        " or ".join([f"'{LaunchConfiguration(name)}' == 'true'" for name, *_ in SHORT_ARGS])
    ])

    # Locate the RealSense package
    realsense_pkg = FindPackageShare("realsense2_camera")

    # Conditionally include RealSense launch
    rs_action = GroupAction(
        condition=IfCondition(any_enabled),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([realsense_pkg, "/launch/rs_launch.py"]),
                launch_arguments={
                    # Enabled streams
                    "enable_color":       LaunchConfiguration("camera"),
                    "enable_depth":       LaunchConfiguration("depth"),
                    "enable_infra1":      LaunchConfiguration("ir_left"),
                    "enable_infra2":      LaunchConfiguration("ir_right"),
                    "enable_ir_emitter":  LaunchConfiguration("ir_projector"),
                    "enable_accel":       LaunchConfiguration("rs_imu"),
                    "enable_gyro":        LaunchConfiguration("rs_imu"),
                    "enable_sync":        "true",

                    # RGB camera settings
                    "rgb_camera.color_profile":         "640x360x15",
                    "rgb_camera.format":                "BGR8",
                    "rgb_camera.enable_auto_exposure":  "True",
                    "rgb_camera.backlight_compensation":"False",
                    "rgb_camera.enable_auto_white_balance": "True",

                    # Depth settings
                    "depth_module.depth_profile": "640x480x15",
                    "enable_rgbd": "False",

                    # IMU method
                    "unite_imu_method": "2",
                }.items()
            )
        ]
    )

    ld.add_action(rs_action)
    return ld
