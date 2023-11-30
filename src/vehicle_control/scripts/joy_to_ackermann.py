#!/usr/bin/env python

import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import Joy

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
        self.joy_sub = rospy.Subscriber('/rc/joy', Joy, self.callback)

    def callback(self, msg):

        # deadman button
        if msg.buttons[self.dead_btn] == self.manu_val:

            # steering
            steer_val = msg.axes[self.steer_ax] * self.steer_scale

            if steer_val > self.steer_deadzone:
                steer_rad = steer_val * self.servo_max
            elif steer_val < -self.steer_deadzone:
                steer_rad = steer_val * self.servo_min * (-1)
            else:
                steer_rad = self.servo_mid

            # throttle
            speed_val = msg.axes[self.speed_ax]

            if speed_val > self.speed_deadzone:
                speed_mps = speed_val * self.speed_max
            elif speed_val < -self.speed_deadzone:
                speed_mps = speed_val * self.speed_min * (-1)
            else:
                speed_mps = self.speed_mid


            self.ackMsg.header.stamp = rospy.Time.now()
            self.ackMsg.drive.steering_angle = steer_rad
            self.ackMsg.drive.speed = speed_mps

            self.ackermann_pub.publish(self.ackMsg)


    def load_params(self):
      
        # load parameters

        if self.control_type == 'rc':
            self.steer_ax = rospy.get_param("/rc_steering_axis")
            self.speed_ax = rospy.get_param("/rc_speed_axis")
            self.dead_btn = rospy.get_param("/rc_deadman_button")
            self.manu_val = rospy.get_param("/rc_manu_value")
            self.steer_scale = 1

        elif self.control_type == 'joy':
            self.steer_ax = rospy.get_param("/joy_steering_axis")
            self.speed_ax = rospy.get_param("/joy_speed_axis")
            self.dead_btn = rospy.get_param("/joy_deadman_button")
            self.steer_scale = -1 # switch directions

        self.servo_min = rospy.get_param("/servo_rad_min")
        self.servo_mid = rospy.get_param("/servo_rad_mid")
        self.servo_max = rospy.get_param("/servo_rad_max")

        self.speed_min = rospy.get_param("/speed_min")
        self.speed_mid = rospy.get_param("/speed_mid")
        self.speed_max = rospy.get_param("/speed_max")

        self.speed_deadzone = rospy.get_param("/speed_deadzone")
        self.steer_deadzone = rospy.get_param("/steer_deadzone")

if __name__ == '__main__':

    # initialize node
    rospy.init_node('joy_control', anonymous=True)

    ctrl = JoyControl()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")