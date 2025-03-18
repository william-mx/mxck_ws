import rclpy
from rclpy.clock import Clock

from tf2_ros import Buffer, TransformListener
from transforms3d.quaternions import quat2mat
from geometry_msgs.msg import PoseStamped, Quaternion
import numpy as np
import struct
import cv2
from nav_msgs.msg import Path
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField, CompressedImage
from cv_bridge import CvBridge
from ackermann_msgs.msg import AckermannDriveStamped

def create_ackermann_msg(speed, steering_angle, timestamp=None):
    msg = AckermannDriveStamped()
    
    msg.header.stamp = timestamp.to_msg() if timestamp else Clock().now().to_msg()

    msg.drive.speed = speed
    msg.drive.steering_angle = steering_angle
    
    return msg

def create_compressed_grayscale_image_message(cv_image, timestamp=None):
    """
    Create a ROS 2 CompressedImage message from an OpenCV grayscale image.

    Args:
        cv_image (numpy.ndarray): The OpenCV image to be compressed. 
                                  It should be in the format (height, width) for grayscale.
        timestamp (rclpy.time.Time, optional): The timestamp for the header of the message.
                                               If None, uses the current ROS 2 time.

    Returns:
        CompressedImage: A ROS 2 CompressedImage message containing the compressed grayscale image data.
    """
    if len(cv_image.shape) == 3:
        # Convert to grayscale if it's not already
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

    compressed_image_msg = CompressedImage()
    compressed_image_msg.header.stamp = timestamp.to_msg() if timestamp else Clock().now().to_msg()

    compressed_image_msg.format = "jpeg"  # JPEG supports grayscale
    compressed_image_msg.data = np.array(cv2.imencode('.jpg', cv_image, [cv2.IMWRITE_JPEG_QUALITY, 100])[1]).tobytes()
    
    return compressed_image_msg

def create_compressed_image_message(cv_image, timestamp=None):
    """
    Create a ROS 2 CompressedImage message from an OpenCV image.

    Args:
        cv_image (numpy.ndarray): The OpenCV image to be compressed. 
                                  It should be in the format (height, width, channels).
        timestamp (rclpy.time.Time, optional): The timestamp for the header of the message.
                                               If None, uses the current ROS 2 time.

    Returns:
        CompressedImage: A ROS 2 CompressedImage message containing the compressed image data.
    """
    compressed_image_msg = CompressedImage()

    compressed_image_msg.header.stamp = timestamp.to_msg() if timestamp else Clock().now().to_msg()

    compressed_image_msg.format = "jpeg"
    compressed_image_msg.data = np.array(cv2.imencode('.jpg', cv_image)[1]).tobytes()
    
    return compressed_image_msg


def create_ros_image(numpy_image: np.ndarray, timestamp=None):
    """
    Converts a NumPy image to a ROS 2 Image message.
    
    Args:
        numpy_image (np.ndarray): Image in NumPy array format (H x W x C).
        timestamp (builtin_interfaces.msg.Time, optional): Timestamp for the message.
    
    Returns:
        sensor_msgs.msg.Image: ROS 2 Image message.
    """
    bridge = CvBridge()
    ros_image = bridge.cv2_to_imgmsg(numpy_image, encoding="bgr8")

    ros_image.header.stamp = timestamp.to_msg() if timestamp else Clock().now().to_msg()

    ros_image.header.frame_id = "camera_frame"  # Set appropriate frame ID
    return ros_image

def angle_to_quaternion(angle):
    """
    Convert a yaw angle (in radians) to a ROS2 Quaternion message.
    
    Parameters
    ----------
    angle : float
        The yaw angle in radians.
    
    Returns
    -------
    Quaternion
        A ROS2 Quaternion message representing the yaw rotation.
    """
    quaternion = Quaternion()
    quaternion.x = 0.0
    quaternion.y = 0.0
    quaternion.z = np.sin(angle / 2.0)
    quaternion.w = np.cos(angle / 2.0)
    return quaternion


def create_pose_message(point, angle, frame_id = 'base_link', timestamp=None):
    """
    Create a PoseStamped message from a point and yaw angle.
    
    Parameters
    ----------
    point : list or np.ndarray
        A list or array containing the [x, y, z] coordinates of the point.
        If z is not provided, it defaults to 0.0.
    angle : float
        The yaw angle in radians.
    
    Returns
    -------
    PoseStamped
        A ROS2 PoseStamped message containing the position and orientation.
    """

    pose = PoseStamped()
    pose.header.stamp = timestamp.to_msg() if timestamp else Clock().now().to_msg()
    pose.header.frame_id = frame_id
    pose.pose.position.x = float(point[0])
    pose.pose.position.y = float(point[1])
    pose.pose.position.z = float(point[2]) if len(point) > 2 else 0.0
    pose.pose.orientation = angle_to_quaternion(angle)
    return pose

def create_path_message(waypoints, frame_id='base_link', timestamp=None):
    """
    Create a ROS2 Path message from a set of waypoints and a frame ID.
    
    Parameters
    ----------
    waypoints : np.ndarray or list
        A numpy array or list of waypoints, where each waypoint is [x, y, theta].
        Each row represents the (x, y) coordinates and the yaw angle (theta) in radians.
        Expected shape: (N, 3), where N is the number of waypoints.
    frame_id : str, optional
        The frame of reference for the path (default is 'base_link').
    timestamp : builtin_interfaces.msg.Time, optional
        The timestamp for the header. If not provided, the current ROS time is used.
    
    Returns
    -------
    Path
        A ROS2 Path message containing PoseStamped messages for each waypoint.
    
    Raises
    -------
    ValueError
        If the waypoints array does not have 3 columns (x, y, theta).
    """
    clock = Clock()
    if timestamp is None:
        timestamp = clock.now().to_msg()

    # Ensure waypoints is a numpy array.
    waypoints = np.array(waypoints)

    # If a single waypoint is provided, reshape it.
    if waypoints.ndim == 1:
        waypoints = waypoints.reshape(1, -1)

    if waypoints.shape[1] != 3:
        raise ValueError("Waypoints should have 3 columns: x, y, and theta.")

    path_msg = Path()
    path_msg.header.stamp = timestamp
    path_msg.header.frame_id = frame_id

    for point in waypoints:
        pose = create_pose_message(point[:2], point[2])
        pose.header.frame_id = frame_id
        path_msg.poses.append(pose)

    return path_msg


def image_msg_to_numpy(image_msg):
    """Convert a ROS sensor_msgs/Image (BGR8) to a NumPy array."""
    try:
        return np.frombuffer(image_msg.data, dtype=np.uint8).reshape(image_msg.height, image_msg.width, 3)
    except Exception as e:
        print(f"Error converting image_msg to NumPy: {e}")
        return None

def compressed_image_msg_to_numpy(compressed_msg):
    """Convert a ROS sensor_msgs/CompressedImage (BGR8) to a NumPy array."""
    try:
        return cv2.imdecode(np.frombuffer(compressed_msg.data, np.uint8), cv2.IMREAD_COLOR)
    except Exception as e:
        print(f"Error converting compressed image_msg to NumPy: {e}")
        return None

def get_relative_transform(source_frame: str, target_frame: str) -> np.ndarray:
    """
    Calculates the relative transformation matrix between two ROS 2 frames using transforms3d.

    This version waits (with retries) for the transform to become available in the tf buffer,
    which helps ensure that transient delays in publishing do not result in missing transforms.

    Args:
        source_frame (str): The source frame.
        target_frame (str): The target frame.

    Returns:
        np.ndarray: A 4x4 transformation matrix (rotation + translation) that transforms points 
                    from the source frame to the target frame, or None if the transform fails.
    """
    # Initialize ROS 2 if it isn't already initialized.
    if not rclpy.ok():
        rclpy.init()

    # Create a temporary ROS node.
    node = rclpy.create_node('get_relative_transform_node')

    # Create a tf2 Buffer and TransformListener.
    # Note: We do NOT use spin_thread=True so that we can manage spinning manually.
    tf_buffer = Buffer(node=node)
    tf_listener = TransformListener(tf_buffer, node)

    # Use a SingleThreadedExecutor to spin the node synchronously.
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)

    # Maximum time to wait for the transform (in seconds).
    max_wait_time = 5.0
    start_time = node.get_clock().now()
    transform = None

    # Retry loop: keep trying until the transform is available or we hit the maximum wait time.
    while (node.get_clock().now() - start_time).nanoseconds * 1e-9 < max_wait_time:
        try:
            transform = tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rclpy.time.Time(),  # Use the latest available transform.
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            # If lookup_transform succeeds, break out of the loop.
            break
        except Exception as e:
            # Spin a little to allow new transform messages to arrive.
            executor.spin_once(timeout_sec=0.1)

    if transform is None:
        node.get_logger().error(
            f"Could not get transform from '{source_frame}' to '{target_frame}' within {max_wait_time} seconds."
        )
        executor.shutdown()
        node.destroy_node()
        # rclpy.shutdown()
        return None


    # Clean up: shut down the executor, destroy the node, and shutdown rclpy.
    executor.shutdown()
    node.destroy_node()
    # rclpy.shutdown()

    return transform

def create_point_cloud_message(points, frame_id='base_link', timestamp=None):
    """
    Create a ROS2 PointCloud2 message from a set of 2D or 3D points.
    """
    points = np.array(points)
    
    if points.shape[1] not in [2, 3]:
        raise ValueError("Points should have 2 or 3 columns representing [x, y] or [x, y, z].")
    
    if points.shape[1] == 2:
        points = np.hstack((points, np.zeros((points.shape[0], 1))))

    pc_msg = PointCloud2()
    pc_msg.header = Header()
    pc_msg.header.stamp = timestamp if timestamp is not None else Clock().now().to_msg()
    pc_msg.header.frame_id = frame_id

    pc_msg.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
    ]
    pc_msg.is_bigendian = False
    pc_msg.point_step = 12
    pc_msg.is_dense = True

    point_data = bytearray()
    for point in points:
        point_data.extend(struct.pack('fff', point[0], point[1], point[2]))

    pc_msg.data = bytes(point_data)
    pc_msg.row_step = pc_msg.point_step * points.shape[0]
    pc_msg.height = 1
    pc_msg.width = points.shape[0]

    return pc_msg
