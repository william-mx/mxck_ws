#!/usr/bin/env python3

import sys
import rospy
import numpy as np
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float64

class JointController:

	def __init__(self):
		self.ackermann_sub = rospy.Subscriber('/ackermann_cmd', AckermannDriveStamped, self.ackermann_callback, queue_size=1)
		self.right_steer_pub = rospy.Publisher('/carbot/right_steer_position_controller/command', Float64, queue_size=1)
		self.left_steer_pub = rospy.Publisher('/carbot/left_steer_position_controller/command', Float64, queue_size=1)
		self.lf_wheel_pub = rospy.Publisher('/carbot/lf_wheel_velocity_controller/command', Float64, queue_size=1)
		self.rf_wheel_pub = rospy.Publisher('/carbot/rf_wheel_velocity_controller/command', Float64, queue_size=1)
		self.lr_wheel_pub = rospy.Publisher('/carbot/lr_wheel_velocity_controller/command', Float64, queue_size=1)
		self.rr_wheel_pub = rospy.Publisher('/carbot/rr_wheel_velocity_controller/command', Float64, queue_size=1)
		self.wheel_speed_msg = Float64()
		self.steer_ang_left_msg = Float64()
		self.steer_ang_right_msg = Float64()
		self.r_dyn = 0.05
		self.l_rad = 0.280
		self.b_rad = 0.100
		
	def ackermann_callback(self, msg):
		steer_ang = msg.drive.steering_angle
		v_veh = msg.drive.speed
		
		self.wheel_speed_msg.data = v_veh/self.r_dyn
		
		if steer_ang != 0:
			R_corner = self.l_rad/np.tan(steer_ang)
			self.steer_ang_right_msg.data = np.arctan(self.l_rad/(R_corner + self.b_rad/2))
			self.steer_ang_left_msg.data = np.arctan(self.l_rad/(R_corner - self.b_rad/2))
		else:
			self.steer_ang_right_msg.data = 0
			self.steer_ang_left_msg.data = 0
		
		self.lf_wheel_pub.publish(self.wheel_speed_msg)
		self.rf_wheel_pub.publish(self.wheel_speed_msg)
		self.lr_wheel_pub.publish(self.wheel_speed_msg)
		self.rr_wheel_pub.publish(self.wheel_speed_msg)
		self.right_steer_pub.publish(self.steer_ang_right_msg)
		self.left_steer_pub.publish(self.steer_ang_left_msg)
		
		
def main(args):
    rospy.init_node('ackermann_to_joints', anonymous=True)
    
    ic = JointController()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main(sys.argv)
