#!/usr/bin/env python3

import os
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import CameraInfo
from foxglove_msgs.msg import ImageMarkerArray
import rospkg
from rospy.exceptions import ROSException

from utils.message_utils import createImageMarkerPointcloud, getRelativTransform

class LidarToImageProjector:
    def __init__(self):
        
        self.camera_info_received = False
        self.K = self.wait_for_camera_info()

        self.T = getRelativTransform('laser', 'camera_rgb')
        
        # Subscribe to the scan topic
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)

        self.pixels_pub = rospy.Publisher('/marker_pointcloud', ImageMarkerArray, queue_size=1)

    def parse_camera_info_message(self, msg):
        # Extract camera intrinsics, width, and height
        self.K = np.array(msg.K).reshape(3,3)
        self.width = msg.width
        self.height = msg.height
        self.camera_info_received = True
        rospy.loginfo("Camera info received!")

    def wait_for_camera_info(self):

        rospy.loginfo("Waiting for /camera/color/camera_info message...")

        # Subscribe to the /camera/color/camera_info topic
        sub = rospy.Subscriber('/camera/color/camera_info', CameraInfo, self.parse_camera_info_message)

        # Wait for the camera info to be received
        while not self.camera_info_received and not rospy.is_shutdown():
            rospy.sleep(1.0)  # Sleep to avoid busy waiting

        # Unregister the subscriber after receiving the message
        sub.unregister()

        return self.K

    
    def scan_callback(self, msg):

        angle_min = msg.angle_min
        angle_max = msg.angle_max
        angle_increment = msg.angle_increment

        # Convert scan data to 3D points
        angles = np.arange(angle_min, angle_max + angle_increment, angle_increment)
        ranges = np.array(msg.ranges)
        valid_indices = np.isfinite(ranges)
        ranges = ranges[valid_indices]
        angles = angles[valid_indices]

        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)
        z = np.zeros_like(x)

        # Combine x, y, z into a single array of shape (N, 3)
        points = np.column_stack((x, y, z))

        # Project points to image
        pixels = self.project_points_to_image(points)

        # Visualize the projected points
        pc_msg = createImageMarkerPointcloud(pixels)

        try:
            self.pixels_pub.publish(pc_msg)
        except ROSException as e:
            rospy.logerr("A ROS exception occurred: %s", str(e))
    
    def project_points_to_image(self, pts):
        # Add homogeneous coordinate
        pts_homo = np.hstack((pts, np.ones((pts.shape[0], 1))))

        # Apply transformation
        pts_transformed = (self.T @ pts_homo.T).T

        # Keep only points with z > 0
        pts_transformed = pts_transformed[pts_transformed[:, 2] > 1e-6]

        # Project to image plane
        pts_2d = self.K @ pts_transformed[:, :3].T
        pixels = (pts_2d[:2] / pts_2d[2]).T

        return pixels


if __name__ == '__main__':

    rospy.init_node('lidar_to_image_projector')
    projector = LidarToImageProjector()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass