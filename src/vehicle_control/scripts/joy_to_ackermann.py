#!/usr/bin/env python

import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import Joy
import numpy as np


def get_interp(x_vals, y_vals):     
   return lambda x: np.interp(x, x_vals, y_vals)

class JoyControl:

    def __init__(self):

        # select control type
        self.control_types = ['joy', 'rc']  # [joystick, remote control]

        # get param, optionally pass in a default value to use if the parameter is not set
        self.control_type = rospy.get_param("control_type", 'rc')

        # check parameter
        if not self.control_type in self.control_types:
            rospy.logerr('Invalid control type parmater. Choose between %s, and %s.' 
            % tuple(self.control_types))
        else:
            rospy.loginfo("control_type: %s", self.control_type)

        # load parameters
        self.load_params()

        # define messages
        self.ackMsg = AckermannDriveStamped()

        # publish ackermann messages to VESC
        self.ackermann_pub = rospy.Publisher('/rc/ackermann_cmd', AckermannDriveStamped, queue_size=1) 

        # subscribe to joy
        if self.control_type == 'rc':
            self.joy_sub = rospy.Subscriber('/rc/joy', Joy, self.callback)
        elif self.control_type == 'joy':
            self.joy_sub = rospy.Subscriber('/joy', Joy, self.callback)

    def callback(self, msg):

        steering_val = msg.axes[self.steer_ax]
        speed_val = msg.axes[self.speed_ax]

        steering_angle = self.steer_mapping(steering_val)
        speed = self.speed_mapping(speed_val)

        self.ackMsg.header.stamp = rospy.Time.now()
        self.ackMsg.drive.steering_angle = steering_angle
        self.ackMsg.drive.speed = speed

        self.ackermann_pub.publish(self.ackMsg)


    def load_params(self):
      
        # load parameters

        if self.control_type == 'rc':
            self.steer_ax = rospy.get_param("/rc_steering_axis")
            self.speed_ax = rospy.get_param("/rc_speed_axis")

        elif self.control_type == 'joy':
            self.steer_ax = rospy.get_param("/joy_steering_axis")
            self.speed_ax = rospy.get_param("/joy_speed_axis")

        steer_max = rospy.get_param("/steering_angle_max")

        max_backward_speed = rospy.get_param("/max_backward_speed")
        max_forward_speed = rospy.get_param("/max_forward_speed")

        # erpm = speed_to_erpm_gain * speed (in m/s)
        # speed = erpm / speed_to_erpm_gain
        erpm_min = rospy.get_param("/erpm_min")
        speed_to_erpm_gain = rospy.get_param("/speed_to_erpm_gain")
        speed_min = erpm_min / speed_to_erpm_gain

        joy_deadzone = rospy.get_param("/joy_deadzone")

        eps = np.finfo(np.float32).eps
        self.steer_mapping = get_interp((-1.0, 0.0, 1.0), (-steer_max, 0, steer_max))
        self.speed_mapping = get_interp((-1.0, -joy_deadzone, -joy_deadzone + eps, joy_deadzone - eps, joy_deadzone, 1.0), \
                                        (max_backward_speed, -speed_min, 0.0, 0.0, speed_min, max_forward_speed))
        
if __name__ == '__main__':

    # initialize node
    rospy.init_node('joy_control', anonymous=True)

    ctrl = JoyControl()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")