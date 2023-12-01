#!/usr/bin/env python

import rospy
import rospkg
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64
from math import atan
import numpy as np
from rc_safety_check import CalibrationWizard

class AckermannToVesc:

    def __init__(self):
        # Load parameters
        self.load_params()

        # Mode: manual, autonomous, deadman
        self.mode = self.dead_val

        # Initialization
        self.init_mapping_function()

        # Define messages
        self.duty_msg = Float64()
        self.servo_msg = Float64()

        # Subscribe to PWM signals from RC receiver
        self.rc_sub = rospy.Subscriber('/rc/ackermann_cmd', AckermannDriveStamped, lambda x: self.callback(x, self.manu_val))
        self.ad_sub = rospy.Subscriber('/autonomous/ackermann_cmd', AckermannDriveStamped, lambda x: self.callback(x, self.auto_val))
        self.joy_sub = rospy.Subscriber('/rc/joy', Joy, self.deadman)

        # Publish commands to VESC driver
        self.duty_pub = rospy.Publisher('/commands/motor/duty_cycle', Float64, queue_size=1)
        self.servo_pub = rospy.Publisher('/commands/servo/position', Float64, queue_size=1)
      
        # force the user to switch to manual mode at program start
        self.init_mode = False
        self.informed = False # print information once
        
    def deadman(self, msg):
        self.mode = msg.buttons[self.dead_btn]
        
        if self.mode == self.manu_val:
           self.init_mode = True

    def callback(self, ackermann_msg, target):

        if not self.init_mode:
            if not self.informed:
               rospy.logwarn("Safety feature: First, switch to manual mode to start the controller!")
               self.informed = True
         
        if self.mode != target:
            return

        steer_rad = ackermann_msg.drive.steering_angle
        speed = ackermann_msg.drive.speed

        # Similar to real vehicles, carkits cannot drive infinitely slow.
        # Set a minimum starting speed for sensorless commutation.
        if abs(speed) > self.speed_clip:
            duty = self.speed_to_duty_gain * speed
        else:
            duty = self.duty_mid

        if abs(steer_rad) > self.rad_max:
            rospy.logwarn("Clipping steering command to +/- %.3f" % self.rad_max)
            steer_rad = max(min(steer_rad, self.rad_max), -self.rad_max)

        if steer_rad > 0:
            val = self.rr_m * steer_rad + self.servo_mid
        else:
            val = self.lr_m * steer_rad + self.servo_mid

        self.servo_msg.data = max(min(val, self.servo_max), self.servo_min)
        self.duty_msg.data = duty

        self.servo_pub.publish(self.servo_msg)
        self.duty_pub.publish(self.duty_msg)

    def load_params(self):
        self.wheelbase = rospy.get_param("/wheelbase")
        self.servo_mid = rospy.get_param("/servo_mid")
        self.servo_max = rospy.get_param("/servo_max")
        self.servo_min = rospy.get_param("/servo_min")
        self.duty_mid = rospy.get_param("/duty_mid")
        self.lr_rmin = rospy.get_param("/lr_rmin")
        self.rr_rmin = rospy.get_param("/rr_rmin")
        self.speed_to_duty_gain = rospy.get_param("/speed_to_duty_gain")
        self.speed_clip = rospy.get_param("/speed_clip")
        self.dead_val = rospy.get_param("/rc_dead_value")  # Deadman switch
        self.auto_val = rospy.get_param("/rc_auto_value")  # Drive autonomously
        self.manu_val = rospy.get_param("/rc_manu_value")  # Drive manually
        self.dead_btn = rospy.get_param("/rc_deadman_button")

        rospy.loginfo("The starting speed is set to +/- %.1f m/s" % self.speed_clip)

    def init_mapping_function(self):
        self.lr_rad_max = atan(self.wheelbase / self.lr_rmin)
        self.rr_rad_max = atan(self.wheelbase / self.rr_rmin)
        self.rr_dy = self.servo_max - self.servo_mid
        self.lr_dy = self.servo_mid - self.servo_min
        self.rr_m = self.rr_dy / self.rr_rad_max
        self.lr_m = self.lr_dy / self.lr_rad_max
        self.rad_max = max(abs(self.rr_rad_max), abs(self.lr_rad_max))

        rospy.loginfo("The maximum steering angle is set to +/- %.3f rad" % self.rad_max)


if __name__ == '__main__':
   rospy.init_node('ackermann_to_vesc', anonymous=True)
   
   # The emergency stop function is a necessary safety feature.
   # Therefore, a safety check is performed first to verify if the RC is properly calibrated
   r = rospkg.RosPack()
   fpath = r.get_path('vehicle_control') + '/config/control_config.yaml'

   wiz = CalibrationWizard(fpath)
   success = wiz.await_calibration_completion()

   if success:
      av = AckermannToVesc()

   try:
      rospy.spin()
   except KeyboardInterrupt:
      print("Shutting down ackermann_to_vesc")
