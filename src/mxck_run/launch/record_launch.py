import os
import time
import random
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

# Example Usage
# ros2 launch mxck_run record_launch.py filename:=custom_bag format:=sqlite3

# List of topics to record
topics = [
    "/rc/ackermann_cmd",
    "/camera/imu",
    "/imu"
]

# Short lists of adjectives and animals for naming
ADJECTIVES = ["brave", "calm", "eager", "gentle", "jolly", "kind", "lively", "merry", "proud", "witty"]
ANIMALS = ["antelope", "beaver", "cheetah", "dolphin", "elephant", "falcon", "giraffe", "hippo", "iguana", "jaguar"]

def generate_random_name():
    """Generate a name in the format 'adjective_animal_number'."""
    adjective = random.choice(ADJECTIVES)
    animal = random.choice(ANIMALS)
    number = random.randint(0, 99)
    return f"{adjective}_{animal}_{number:02}"  # Two-digit number for consistency

def generate_launch_description():
    # Base directory to store bag files
    base_bag_directory = "/mxck2_ws/src/mxck_run/bagfiles"
    os.makedirs(base_bag_directory, exist_ok=True)

    # Declare launch arguments
    bag_filename_arg = DeclareLaunchArgument(
        'filename',
        default_value=generate_random_name(),
        description="Custom filename for the bag file. If empty, a random name is used."
    )

    storage_format_arg = DeclareLaunchArgument(
        'format',
        default_value='mcap',  # Default to MCAP
        description="Storage format for the bag file (options: 'mcap' or 'sqlite3')."
    )

    # Define paths
    bag_directory = PathJoinSubstitution([
        base_bag_directory,
        LaunchConfiguration('filename')
    ])

    # ROS 2 bag record command with dynamic storage format
    record_bag = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-o', bag_directory, '-s', LaunchConfiguration('format')] + topics,
        output='screen'
    )

    return LaunchDescription([
        bag_filename_arg,
        storage_format_arg,
        record_bag
    ])
