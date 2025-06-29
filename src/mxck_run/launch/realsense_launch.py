from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, TimerAction, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import subprocess

# Define concise launch arguments
SHORT_ARGS = [
    ("camera",       "false", "Enable the RGB/color camera"),
    ("rgbd",         "false", "Enable the RGB-D composite stream"),
    ("rs_imu",       "false", "Enable the RealSense IMU (accel + gyro)"),
    ("ir_left",      "false", "Enable the left IR sensor"),
    ("ir_right",     "false", "Enable the right IR sensor"),
    ("ir_projector", "false", "Enable the IR projector pattern emitter (true/false)"),
]

# Runtime callback to set emitter_enabled to 1 or 0
def set_emitter_param(context, *args, **kwargs):
    ir_proj = context.launch_configurations.get("ir_projector", "false")
    emitter_value = "1" if ir_proj == "true" else "0"

    print(f"[Launch] Setting depth_module.emitter_enabled = {emitter_value}")
    subprocess.run([
        "ros2", "param", "set", "/camera/camera",
        "depth_module.emitter_enabled", emitter_value
    ])
    return []

def generate_launch_description():
    ld = LaunchDescription()

    # Declare all arguments
    for name, default, desc in SHORT_ARGS:
        ld.add_action(
            DeclareLaunchArgument(name, default_value=default, description=desc)
        )

    # Locate RealSense package
    realsense_pkg = FindPackageShare("realsense2_camera")

    # Conditionally include the RealSense launch file
    rs_camera = GroupAction(
        condition=IfCondition(
            PythonExpression([
                "'", 
                LaunchConfiguration("camera"), "' == 'true' or '",
                LaunchConfiguration("rgbd"), "' == 'true' or '",
                LaunchConfiguration("rs_imu"), "' == 'true' or '",
                LaunchConfiguration("ir_left"), "' == 'true' or '",
                LaunchConfiguration("ir_right"), "' == 'true' or '",
                LaunchConfiguration("ir_projector"), "' == 'true'"
            ])
        ),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([realsense_pkg, "/launch/rs_launch.py"]),
                launch_arguments={
                    "enable_color":       LaunchConfiguration("camera"),
                    "enable_depth":       LaunchConfiguration("rgbd"),
                    "enable_rgbd":        LaunchConfiguration("rgbd"),
                    "align_depth.enable": LaunchConfiguration("rgbd"),
                    "enable_sync":        LaunchConfiguration("rgbd"),

                    # Infra & IR
                    "enable_infra1":      LaunchConfiguration("ir_left"),
                    "enable_infra2":      LaunchConfiguration("ir_right"),

                    # IMU
                    "enable_accel":       LaunchConfiguration("rs_imu"),
                    "enable_gyro":        LaunchConfiguration("rs_imu"),
                    "unite_imu_method":   "2",

                    # Camera settings
                    "rgb_camera.color_profile":         "640x360x15",
                    "rgb_camera.format":                "BGR8",
                    "rgb_camera.enable_auto_exposure":  "True",
                    "rgb_camera.backlight_compensation":"False",
                    "rgb_camera.enable_auto_white_balance": "True",

                    # Depth profile
                    "depth_module.depth_profile": "640x360x15",
                    "depth_module.infra_profile": "640x360x15",
                }.items()
            )
        ]
    )

    # Add RealSense node
    ld.add_action(rs_camera)

    # Add delayed param set for emitter_enabled
    ld.add_action(
        TimerAction(
            period=3.0,
            actions=[
                OpaqueFunction(function=set_emitter_param)
            ]
        )
    )

    return ld
