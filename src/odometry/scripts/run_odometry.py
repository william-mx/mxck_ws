#!/usr/bin/env python3

import os
import sys
import rospy
import rospkg
import numpy as np
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import Imu
from nav_msgs.msg import Path

r = rospkg.RosPack()
sys.path.append(os.path.join(r.get_path('odometry'), 'include'))

from odom_solver import AckermannOdomSolver, InertialOdomSolver
from utils.message_utils import caratePathMessage, map_cartesian_to_vehicle_frame


class OdometryNode():
    def __init__(self, ground_truth_trajectory = None):
        
        # Initialize the node
        rospy.init_node('odometry', anonymous=True)

        self.ack_odom = AckermannOdomSolver()
        self.imu_odom = InertialOdomSolver()
        self.rs_imu_odom = InertialOdomSolver(speed_gain = 0.4)

        # Subscribe to AckermannDriveStamped messages
        self.ack_subscriber = rospy.Subscriber("/rc/ackermann_cmd", AckermannDriveStamped, self.ackermann_callback)

        # Subscribe to RealSense IMU messages
        self.rs_imu_subscriber = rospy.Subscriber("/rs_imu", Imu, self.rs_imu_callback)
        
        # Subscribe to IMU messages
        self.imu_subscriber = rospy.Subscriber("/imu_calibrated", Imu, self.imu_callback)
        

        self.ack_pub = rospy.Publisher('ack_odometry', Path, queue_size=1)
        self.rs_imu_pub = rospy.Publisher('rs_imu_odometry', Path, queue_size=1)
        self.imu_pub = rospy.Publisher('imu_odometry', Path, queue_size=1)
        self.gt_pub = rospy.Publisher('gt_odometry', Path, queue_size=1, latch=True)


        if not ground_truth_trajectory is None:
            self.gt_path_msg = caratePathMessage(ground_truth_trajectory) 
            self.gt_pub.publish(self.gt_path_msg)

    def _get_imu_data(self, msg):
        accel = msg.linear_acceleration.x
        angular_vel = -np.deg2rad(msg.angular_velocity.z)
        return accel, angular_vel

    def _get_rs_imu_data(self, msg):
        accel = msg.linear_acceleration.z
        angular_vel = msg.angular_velocity.y
        return accel, angular_vel
    
    def rs_imu_callback(self, msg):
        return self._imu_callback(msg, fun = self._get_rs_imu_data, every_nth = 60, publisher = self.rs_imu_pub, solver = self.rs_imu_odom)
    
    def imu_callback(self, msg):
        return self._imu_callback(msg, fun = self._get_imu_data, every_nth = 10, publisher = self.imu_pub, solver = self.imu_odom)
    
    def _imu_callback(self, msg, fun, every_nth, publisher, solver):

        accel, angular_vel = fun(msg)
        seq = msg.header.seq

        dt = solver.get_delta_t(msg)

        waypoints = solver.update_pose(accel, angular_vel, dt) # waypoints (np.ndarray): An N x 3 array where each row is [x, y, theta].

        if seq % every_nth == 0:
            # Transform coordinates from Cartesian (x: right, y: forward) to the vehicle's reference frame (x: forward, y: left)
            waypoints = map_cartesian_to_vehicle_frame(waypoints)
            waypoints[:,2] = -waypoints[:,2] - np.pi/2 # FIX IT

            pts = waypoints[::every_nth]
            path_msg = caratePathMessage(pts) 
            publisher.publish(path_msg)

    def ackermann_callback(self, msg):

        """
        Callback function for AckermannDriveStamped messages.
        
        Args:
            msg (AckermannDriveStamped): The incoming message containing steering angle and speed.
        """
        # Extract speed and steering angle from the message
        every_nth = 10 # publish every nth topic
        speed = msg.drive.speed
        steering_angle = msg.drive.steering_angle
        seq = msg.header.seq

        dt = self.ack_odom.get_delta_t(msg)

        waypoints = self.ack_odom.update_pose(speed, steering_angle, dt) # waypoints (np.ndarray): An N x 3 array where each row is [x, y, theta].

        if seq % every_nth == 0:
            waypoints = map_cartesian_to_vehicle_frame(waypoints)
            waypoints[:,2] = -waypoints[:,2] - np.pi/2 # FIX IT

            pts = waypoints[::every_nth]
            path_msg = caratePathMessage(pts) 
            self.ack_pub.publish(path_msg)

if __name__ == "__main__":

    r = rospkg.RosPack()
    bagfile_dir = os.path.join(r.get_path('odometry'), 'bagfiles')
    ground_truth_filepath = os.path.join(bagfile_dir, 'racetrack_polyline_xy_theta.npy')

    if os.path.exists(ground_truth_filepath):
        ground_truth_trajectory = map_cartesian_to_vehicle_frame(np.load(ground_truth_filepath))
    else:
        print("File not found: %s" % ground_truth_filepath)
        ground_truth_trajectory = None

    odometry_node = OdometryNode(ground_truth_trajectory)

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
