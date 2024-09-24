#!/usr/bin/env python3

import cv2
import rospy
import tf2_ros
import numpy as np
from geometry_msgs.msg import PoseStamped, Quaternion, Point, TransformStamped
from nav_msgs.msg import Path
from std_msgs.msg import ColorRGBA, Header
from foxglove_msgs.msg import ImageMarkerArray
from visualization_msgs.msg import ImageMarker
from sensor_msgs.msg import CompressedImage, PointCloud2
from tf.transformations import quaternion_matrix


def crateColorRGBA(color=None):
    """
    Creates a ColorRGBA message for ROS visualization based on the given color code.

    Parameters:
    ----------
    color : str, optional
        A single-character string representing the color. 
        Supported colors:
        - 'r': Red
        - 'g': Green
        - 'b': Blue
        - 'y': Yellow
        - 'm': Magenta
        - 'c': Cyan
        - 'w': White
        - 'k': Black
        Default is 'r' (Red).

    Returns:
    -------
    ColorRGBA
        ROS ColorRGBA message representing the color with (r, g, b, a) values.

    Raises:
    -------
    ValueError:
        If an invalid color code is passed.

    Notes:
    ------
    The color mapping is done using a dictionary where each color code maps to a 4-element tuple 
    (r, g, b, a), where r, g, b are the red, green, blue channels, and a is the alpha (transparency) value.
    """

    color_map = {
        'r': (1.0, 0.0, 0.0, 1.0),  # Red
        'g': (0.0, 1.0, 0.0, 1.0),  # Green
        'b': (0.0, 0.0, 1.0, 1.0),  # Blue
        'y': (1.0, 1.0, 0.0, 1.0),  # Yellow
        'm': (1.0, 0.0, 1.0, 1.0),  # Magenta
        'c': (0.0, 1.0, 1.0, 1.0),  # Cyan
        'w': (1.0, 1.0, 1.0, 1.0),  # White
        'k': (0.0, 0.0, 0.0, 1.0)   # Black
    }

    RGBA = color_map.get(color, color_map['r'])  # Default to red if color is not found

    return ColorRGBA(*RGBA)


def createImageMarkerPointcloud(points, color='r'):
    """
    Creates an ImageMarkerArray message for a point cloud, using the given 2D points and color.

    Parameters:
    ----------
    points : numpy.ndarray (N, 2)
        Array of 2D points (x, y) to be used as the point cloud.
    
    color : str, optional
        A single-character string representing the color. 
        Supported colors:
        - 'r': Red
        - 'g': Green
        - 'b': Blue
        - 'y': Yellow
        - 'm': Magenta
        - 'c': Cyan
        - 'w': White
        - 'k': Black
        Default is 'r' (Red).

    Returns:
    -------
    ImageMarkerArray
        ROS message containing an ImageMarker with the specified points and color.

    Raises:
    -------
    ValueError:
        If the points array is not in the expected shape (N, 2).
    
    Notes:
    ------
    The function expects a 2D points array with shape (N, 2). The points are converted 
    into ROS `Point` messages and added to an ImageMarkerArray as `ImageMarker.POINTS`.
    
    Example Usage:
    --------------
    points = np.array([[1.0, 2.0], [2.5, 3.6], [4.1, 5.0]])
    marker_array = createImageMarkerPointcloud(points, color='b')
    """

    # Validate the shape of the points array
    if not (points.ndim == 2 and points.shape[1] == 2):
        raise ValueError(f"Expected shape (N,2), but got {points.shape}")
    
    # Create an ImageMarkerArray
    markers = ImageMarkerArray()

    # Convert 2D points into ROS Point objects
    marker_points = [Point(x, y, 0) for (x, y) in points]

    # Get the color in ColorRGBA format
    clr = crateColorRGBA(color)

    # Create the ImageMarker and append to the ImageMarkerArray
    markers.markers.append(
        ImageMarker(
            header=Header(stamp=rospy.Time.now()),
            scale=1,
            type=ImageMarker.POINTS,
            points=marker_points,
            outline_color=clr,
        )
    )

    return markers


        
def createImageMarkerBBox(xyxy=None, cxcywh=None, xywh=None, xyxyxyxy=None, color='r'):
    """
    Convert bounding box coordinates into an ImageMarkerArray format compatible with ROS visualization.

    Parameters:
    ----------
    xyxy : numpy.ndarray (N, 4), optional
        Bounding boxes in (xmin, ymin, xmax, ymax) format.
    
    cxcywh : numpy.ndarray (N, 4), optional
        Bounding boxes in (cx, cy, w, h) format.
    
    xywh : numpy.ndarray (N, 4), optional
        Bounding boxes in (xmin, ymin, width, height) format.
    
    xyxyxyxy : numpy.ndarray (N, 4, 2), optional
        Pre-calculated corners for bounding boxes in (x, y) pairs for each corner.
    
    color : str, optional
        The color of the bounding box marker outline, default is "r".
        Choose between 'r', 'g', 'b', 'c', 'm', 'y', 'k', 'w'.

    Returns:
    -------
    ImageMarkerArray
        ROS message containing ImageMarker objects that represent the bounding boxes.

    Raises:
    -------
    ValueError:
        If the input bounding box dimensions do not match the expected format for any input type.

    Notes:
    ------
    The function expects one of the following input formats:
    - xyxy : (xmin, ymin, xmax, ymax)
    - cxcywh : (center_x, center_y, width, height)
    - xywh : (xmin, ymin, width, height)
    - xyxyxyxy : pre-defined corners (x1, y1, x2, y2, x3, y3, x4, y4)
    
    Example Usage:
    --------------
    bbox_xyxy = np.array([[50, 50, 100, 100]])
    marker_array = createImageMarkerBBox(xyxy=bbox_xyxy)
    """

    # Define internal helper function to get corner points in 3D
    def _get_corners(xmin, ymin, xmax, ymax):
        """
        Stack and create corner points in 3D space (x, y, z=0)
        for a given bounding box.
        """
        pts = np.stack([
            np.hstack([xmin, ymin]),  # Top-left
            np.hstack([xmax, ymin]),  # Top-right
            np.hstack([xmax, ymax]),  # Bottom-right
            np.hstack([xmin, ymax])   # Bottom-left
        ], axis=1)  # (N, 4, 2)

        zeros = np.zeros((len(xmin), 4, 1))  # (N, 4, 1) add z=0
        pts = np.concatenate([pts, zeros], axis=-1)  # (N, 4, 3)

        return pts

    # Check bounding box format and convert to corner points
    if xyxy is not None:
        bboxs = xyxy
        if not (bboxs.ndim == 2 and bboxs.shape[1] == 4):
            raise ValueError(f"Expected shape (N,4), but got {bboxs.shape}")
        
        xmin, ymin, xmax, ymax = np.hsplit(bboxs, 4)
        corners = _get_corners(xmin, ymin, xmax, ymax)
    
    elif cxcywh is not None:
        bboxs = cxcywh
        if not (bboxs.ndim == 2 and bboxs.shape[1] == 4):
            raise ValueError(f"Expected shape (N,4), but got {bboxs.shape}")
        
        cx, cy, w, h = np.hsplit(bboxs, 4)

        xmin = cx - w / 2
        xmax = cx + w / 2
        ymin = cy - h / 2
        ymax = cy + h / 2

        corners = _get_corners(xmin, ymin, xmax, ymax)

    elif xyxyxyxy is not None:
        bboxs = xyxyxyxy
        if not (bboxs.ndim == 3 and bboxs.shape[1:] == (4, 2)):
            raise ValueError(f"Expected shape (N,4,2), but got {bboxs.shape}")

        zeros = np.zeros((len(bboxs), 4, 1))  # (N, 4, 1) add z=0
        corners = np.concatenate([bboxs, zeros], axis=-1)  # (N, 4, 3)

    elif xywh is not None:
        bboxs = xywh
        if not (bboxs.ndim == 2 and bboxs.shape[1] == 4):
            raise ValueError(f"Expected shape (N,4), but got {bboxs.shape}")
        
        xmin, ymin, w, h = np.hsplit(bboxs, 4)

        xmax = xmin + w
        ymax = ymin + h

        corners = _get_corners(xmin, ymin, xmax, ymax)

    else:
        raise ValueError("At least one bounding box format must be provided")

    # Create an ImageMarkerArray
    markers = ImageMarkerArray()

    clr = crateColorRGBA(color)

    # Create ImageMarker for each bounding box and add to array
    for box in corners:
        points = [Point(*pt) for pt in box]

        markers.markers.append(
            ImageMarker(
                header=Header(stamp=rospy.Time.now()),
                scale=1,
                type=ImageMarker.POLYGON,
                outline_color=clr,
                points=points,
            )
        )

    return markers


def caratePathMessage(waypoints, frame_id='base_link', timestamp=None):
    """
    Create a ROS Path message from a set of waypoints and a frame ID.

    Parameters:
    ----------
    waypoints : np.ndarray or list
        A numpy array or list of waypoints, where each waypoint is [x, y, theta].
        - Each row represents the (x, y) coordinates and the yaw angle (theta) in radians.
        - Expected shape: (N, 3), where N is the number of waypoints.

    frame_id : str, optional
        The frame of reference for the path (default is 'base_link').

    Returns:
    -------
    Path
        A ROS Path message containing PoseStamped messages for each waypoint.

    Raises:
    -------
    ValueError:
        If the waypoints array does not have 3 columns (x, y, theta).

    Notes:
    ------
    The function first creates PoseStamped messages from each waypoint. The yaw angle (theta) 
    is converted to a quaternion for the pose orientation. All poses are added to a ROS Path message.

    Example Usage:
    --------------
    waypoints = np.array([[1.0, 2.0, 0.5], [2.5, 3.6, 1.2], [4.1, 5.0, -0.3]])
    path_msg = caratePathMessage(waypoints, frame_id='map')
    """

    def angle_to_quaternion(angle):
        """
        Convert a yaw angle (in radians) to a ROS Quaternion message.

        Parameters:
        ----------
        angle : float
            The yaw angle in radians.

        Returns:
        -------
        Quaternion
            A ROS Quaternion message representing the yaw rotation.
        """
        quaternion = Quaternion()
        quaternion.x = 0.0
        quaternion.y = 0.0
        quaternion.z = np.sin(angle / 2.0)
        quaternion.w = np.cos(angle / 2.0)
        return quaternion

    def create_pose(point, angle):
        """
        Create a PoseStamped message from a point and yaw angle.

        Parameters:
        ----------
        point : list or np.ndarray
            A list or array containing the [x, y, z] coordinates of the point.
            If z is not provided, it defaults to 0.0.

        angle : float
            The yaw angle in radians.

        Returns:
        -------
        PoseStamped
            A ROS PoseStamped message containing the position and orientation.
        """
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = float(point[0])
        pose.pose.position.y = float(point[1])
        pose.pose.position.z = float(point[2]) if len(point) > 2 else 0.0
        pose.pose.orientation = angle_to_quaternion(angle)
        return pose

    # Ensure waypoints is a numpy array
    waypoints = np.array(waypoints)

    # Check if input is a single waypoint and reshape if necessary
    if waypoints.ndim == 1:
        waypoints = waypoints.reshape(1, -1)

    # Ensure waypoints have 3 columns (x, y, theta)
    if waypoints.shape[1] != 3:
        raise ValueError("Waypoints should have 3 columns: x, y, and theta.")

    # Create Path message
    path_msg = Path()
    path_msg.header.stamp = timestamp if timestamp is not None else rospy.Time.now()
    path_msg.header.frame_id = frame_id

    # Convert each waypoint to a PoseStamped message and add it to the path
    for point in waypoints:
        pose = create_pose(point[:2], point[2])
        pose.header.frame_id = frame_id
        path_msg.poses.append(pose)

    return path_msg


def createPointCloudMessage(points, frame_id='base_link', timestamp=None):
    """
    Create a ROS PointCloud2 message from a set of 2D or 3D points.

    Parameters:
    ----------
    points : np.ndarray
        A numpy array of shape (N, 2) or (N, 3), where N is the number of points.
        - If shape is (N, 2), the z-coordinate is set to zero.
        - If shape is (N, 3), the z-coordinate is taken from the array.

    frame_id : str, optional
        The frame of reference for the point cloud (default is 'base_link').

    Returns:
    -------
    PointCloud2
        A ROS PointCloud2 message containing the provided points.

    Raises:
    -------
    ValueError:
        If the points array does not have 2 or 3 columns.

    Notes:
    ------
    This function converts a numpy array of 2D or 3D points into a ROS PointCloud2 message.
    It sets the frame ID and populates the x, y, and z coordinates in the point cloud message.

    Example Usage:
    --------------
    points = np.array([[1.0, 2.0], [2.5, 3.6], [4.1, 5.0]])
    pointcloud_msg = createPointCloudMessage(points, frame_id='map')
    """
    
    import struct
    from sensor_msgs.msg import PointCloud2, PointField
    from std_msgs.msg import Header
    
    # Ensure points is a numpy array
    points = np.array(points)
    
    # Check if input is either 2D or 3D points
    if points.shape[1] not in [2, 3]:
        raise ValueError("Points should have 2 or 3 columns representing [x, y] or [x, y, z].")
    
    # Add z=0.0 if only (N, 2) points are provided
    if points.shape[1] == 2:
        points = np.hstack((points, np.zeros((points.shape[0], 1))))
    
    # Define PointCloud2 message
    pc_msg = PointCloud2()
    pc_msg.header = Header()
    pc_msg.header.stamp = timestamp if timestamp is not None else rospy.Time.now()
    pc_msg.header.frame_id = frame_id

    # Define fields (x, y, z)
    pc_msg.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
    ]
    pc_msg.is_bigendian = False
    pc_msg.point_step = 12  # 3 floats * 4 bytes per float
    pc_msg.is_dense = True

    # Convert point data to byte format
    point_data = bytearray()
    for point in points:
        point_data.extend(struct.pack('fff', point[0], point[1], point[2]))
    
    # Populate the PointCloud2 message
    pc_msg.data = bytes(point_data)
    pc_msg.row_step = pc_msg.point_step * points.shape[0]
    pc_msg.height = 1  # Assuming a single row
    pc_msg.width = points.shape[0]

    return pc_msg

def createCompressedImageMessage(cv_image, timestamp=None):
    """
    Create a ROS CompressedImage message from an OpenCV image.

    Args:
        cv_image (numpy.ndarray): The OpenCV image to be compressed. 
                                  It should be in the format (height, width, channels).
        timestamp (rospy.Time, optional): The timestamp for the header of the message.
                                            If None, uses rospy.Time.now().

    Returns:
        CompressedImage: A ROS CompressedImage message containing the compressed image data.
    """
    compressed_image_msg = CompressedImage()
    compressed_image_msg.header.stamp = timestamp if timestamp is not None else rospy.Time.now()
    compressed_image_msg.format = "jpeg"
    compressed_image_msg.data = np.array(cv2.imencode('.jpg', cv_image)[1]).tobytes()
    return compressed_image_msg

def map_cartesian_to_vehicle_frame(waypoints):
    """
    This function changes the waypoints representing positions on the ground from the Cartesian coordinate system to the vehicle's coordinate system.
    It's specifically meant for points that are relevant to the vehicle's movement, like the paths or lane markings.
    The transformation is achieved by rotating these points 90 degrees counterclockwise to align them with how the vehicle perceives its environment.

    In the Cartesian coordinate system: (x-right, y-forward, z-ignored)
    In the vehicle coordinate system: (x-forward, y-left, z-ignored)

    Args:
        waypoints (np.ndarray): An N x 3 array where each row represents [x, y, theta]
                                in the Cartesian frame.

    Returns:
        np.ndarray: An N x 3 array of transformed waypoints [x', y', theta']
                    in the vehicle frame.
    """
    # Rotation matrix to transform from Cartesian to vehicle frame
    # 90 degree counterclockwise rotation
    R = np.array([[0, 1], [-1, 0]])

    # Separate positions and angles
    xy = waypoints[:, :2]  # x and y
    angles = waypoints[:, 2]  # theta

    # Transform positions
    transformed_xy = (R @ xy.T).T  # (N, 2)

    # Compute headings for transformation
    headings = np.array([np.cos(angles), np.sin(angles)])  # (2, N)
    transformed_heading = R @ headings  # (2, N)
    x, y = transformed_heading  # (N,)

    # Transform angles
    transformed_angles = np.arctan2(y, x)  # (N,)

    # Combine transformed positions and angles
    transformed_waypoints = np.column_stack((transformed_xy, transformed_angles)) # (N,3)

    return transformed_waypoints


def getRelativTransform(source_frame: str, target_frame: str) -> np.ndarray:
    """
    Calculate the relative transformation matrix between two ROS frames.

    Args:
        source_frame (str): The source frame.
        target_frame (str): The target frame.

    Returns:
        np.ndarray: 4x4 transformation matrix (rotation + translation) that transforms points from the source frame to the target frame.
    """
    # Create a tf buffer and listener
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    try:
        # Lookup the transform from the target frame to the source frame
        transform: TransformStamped = tf_buffer.lookup_transform(target_frame, source_frame, rospy.Time(0), rospy.Duration(1.0))

        # Extract translation
        translation = np.array([transform.transform.translation.x,
                                transform.transform.translation.y,
                                transform.transform.translation.z])

        # Extract rotation (quaternion)
        rotation = [transform.transform.rotation.x,
                    transform.transform.rotation.y,
                    transform.transform.rotation.z,
                    transform.transform.rotation.w]
        
        # Convert quaternion to a 3x3 rotation matrix
        rotation_matrix = quaternion_matrix(rotation)[:3, :3]

        # Create a 4x4 transformation matrix
        transformation_matrix = np.identity(4)
        transformation_matrix[:3, :3] = rotation_matrix
        transformation_matrix[:3, 3] = translation

        return transformation_matrix

    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        rospy.logerr(f"Could not transform between {source_frame} and {target_frame}: {str(e)}")
        return None



if __name__ == '__main__':

    rospy.init_node('message_utils', anonymous=True)

    mat = getRelativTransform('laser', 'camera_rgb')
    print(mat.round(3))
    # Publishers
    pointcloud_pub = rospy.Publisher('/pointcloud', PointCloud2, queue_size=10)
    marker_pointcloud_pub = rospy.Publisher('/marker_pointcloud', ImageMarkerArray, queue_size=10)
    marker_bbox_pub = rospy.Publisher('/bboxs', ImageMarkerArray, queue_size=10)
    path_pub = rospy.Publisher('/path', Path, queue_size=10)

    # Dummy data
    points = np.random.rand(10, 3)
    pixels = np.random.randint(50, 200, (30, 2))
    xyxy = np.array([[50, 50, 100, 100]])
    waypoints = np.array([[i/10, i/10, (np.pi/2)/i] for i in range(1,10)])
    waypoints = map_cartesian_to_vehicle_frame(waypoints)
    cv_image = np.zeros((480, 640, 3), dtype=np.uint8)
    compressed_image_pub = rospy.Publisher('/camera/color/image_jpeg', CompressedImage, queue_size=10)

    try:
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():


            # Publish dummy data
            pointcloud_pub.publish(createPointCloudMessage(points))
            marker_pointcloud_pub.publish(createImageMarkerPointcloud(pixels))
            marker_bbox_pub.publish(createImageMarkerBBox(xyxy))
            path_pub.publish(caratePathMessage(waypoints))
            compressed_image_pub.publish(createCompressedImageMessage(cv_image))

            rate.sleep()

    except rospy.ROSInterruptException:
        pass