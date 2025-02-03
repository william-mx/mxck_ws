import struct
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from rclpy.clock import Clock
import rclpy
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
from geometry_msgs.msg import TransformStamped
from transforms3d.quaternions import quat2mat
import time


import time
import transforms3d

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
