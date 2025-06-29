from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch_ros.actions import Node
import subprocess


# Define the whitelist as a proper list
TOPIC_WHITELIST = [
    # === Base Topics ===
    "/tf_static",
    "/rc/ackermann_cmd",
    "/autonomous/ackermann_cmd",
    "/scan",

    # === Micro-ROS Topics ===
    "/imu",
    "/uss_sensors",
    "/veh_remote_ctrl",

    # === Camera Topics ===
    "/camera/camera/color/image_raw",
    "/camera/color/image_jpeg",
    "/camera/camera/imu",
    "/camera/camera/depth/image_rect_raw",
    "/camera/camera/infra1/image_rect_raw",
    "/camera/camera/infra2/image_rect_raw",
    "/camera/camera/depth/color/points",
    "/camera/camera/rgbd",

    # === Custom Topics ===
    "/pdc",
    "/pose",
    "/path",
    "/result",
    "/waypoint",
]

# Convert the list to a string representation
WHITELIST_STRING = str(TOPIC_WHITELIST).replace("'", '"')  # Ensure double quotes for XML compatibility

def generate_launch_description():
    # List of launch arguments
    launch_arguments = [
        ("run_foxglove", "false", "Flag to run Foxglove bridge"),
        ("run_camera", "false", "Flag to run RGB camera"),
        ("run_lidar", "false", "Flag to run LiDAR"),
        ("run_micro", "false", "Flag to run micro-ROS"),
        ("broadcast_tf", "false", "Flag to broadcast TFs"),
        ("run_motors", "false", "Flag to start manual control"),
        ("run_rs_imu", "false", "Flag to enable RealSense IMU")
    ]

    # Declare launch arguments dynamically
    declare_launch_arguments = [
        DeclareLaunchArgument(name, default_value=default, description=desc)
        for name, default, desc in launch_arguments
    ]

    # Package Paths
    realsense_pkg = FindPackageShare("realsense2_camera")
    lidar_pkg = FindPackageShare("rplidar_ros")
    micro_ros_pkg = "micro_ros_agent"
    tf_broadcast_pkg = FindPackageShare("mxck_run")
    foxglove_pkg = FindPackageShare("foxglove_bridge")
    motors_pkg = FindPackageShare("vehicle_control")

    # Conditionally include the Foxglove launch file
    foxglove_launch = GroupAction(
        condition=IfCondition(LaunchConfiguration("run_foxglove")),
        actions=[
            IncludeLaunchDescription(
                XMLLaunchDescriptionSource([foxglove_pkg, "/launch/foxglove_bridge_launch.xml"]),
                launch_arguments={
                    "port": "8765",
                    "send_buffer_limit": "10000000",
                    "topic_whitelist": WHITELIST_STRING
                }.items()
            )
        ]
    )


    # Conditionally include the RealSense launch file
    rs_camera = GroupAction(
        condition=IfCondition(
            PythonExpression([
                "'", LaunchConfiguration("run_camera"), "' == 'true' or '",
                LaunchConfiguration("run_rs_imu"), "' == 'true'"
            ])
        ),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([realsense_pkg, "/launch/rs_launch.py"]),
                launch_arguments={
                    "enable_color": LaunchConfiguration("run_camera"),
                    "rgb_camera.color_profile": "640x360x15",
                    "rgb_camera.format": "BGR8",
                    "rgb_camera.enable_auto_exposure": "True",
                    "rgb_camera.backlight_compensation": "False",
                    "rgb_camera.enable_auto_white_balance": "True",
                    "enable_depth": "False",
                    "depth_module.depth_profile": "640x480x15",
                    "enable_accel": LaunchConfiguration("run_rs_imu"),
                    "enable_gyro": LaunchConfiguration("run_rs_imu"),
                    "enable_sync": LaunchConfiguration("run_rs_imu"),
                    "enable_infra1": "False",
                    "enable_infra2": "False",
                    "enable_rgbd": "False",
                    "unite_imu_method": "2",
                }.items()
            )
        ]
    )

    # Conditionally include the LiDAR launch file
    lidar_launch = GroupAction(
        condition=IfCondition(LaunchConfiguration("run_lidar")),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([lidar_pkg, "/launch/rplidar_a2m12_launch.py"])
            )
        ]
    )

    # Conditionally start micro-ROS agent
    micro_ros_agent = Node(
        package=micro_ros_pkg,
        name="micro_ros_agent",
        executable="micro_ros_agent",
        arguments=["serial", "-b", "921600", "--dev", "/dev/stm32_nucleo"],
        respawn=True,
        respawn_delay=20.0,
        condition=IfCondition(LaunchConfiguration("run_micro")),
    )

    # Conditionally include TF broadcaster
    tf_broadcast = GroupAction(
        condition=IfCondition(LaunchConfiguration("broadcast_tf")),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([tf_broadcast_pkg, "/launch/broadcast_tf_launch.py"])
            )
        ]
    )

    # Conditionally include the motors launch file
    motors_launch = GroupAction(
        condition=IfCondition(LaunchConfiguration("run_motors")),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([motors_pkg, "/launch/manual_control_launch.py"])
            )
        ]
    )

    ld = LaunchDescription(declare_launch_arguments)

    # ===== Launch missing ROS2 nodes =====

    # Get list of currently running nodes
    result = subprocess.run(['ros2', 'node', 'list'], stdout=subprocess.PIPE, text=True)
    
    # Get node names and remove the leading slash
    running_nodes = [node.lstrip('/') for node in result.stdout.strip().split('\n')] if result.stdout else []

    # Map node name strings to their executables
    node_names = {
        'foxglove_bridge': foxglove_launch,
        'camera/camera': rs_camera,
        'micro_ros_agent': micro_ros_agent,
        'rplidar_node': lidar_launch,
    }

    # Launch node if it's not already running
    for name, node in node_names.items():
        if not name in running_nodes:
            ld.add_action(node)

    ld.add_action(tf_broadcast)
    ld.add_action(motors_launch)

    return ld
