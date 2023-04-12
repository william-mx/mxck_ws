#!/usr/bin/env python

import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float64
from math import atan

class AckermannToVesc:

   def __init__(self):

      # load parameters
      self.load_params()

      # init
      self.init_mapping_function()

      # define messages
      self.erpm_msg = Float64()
      self.servo_msg = Float64()

      # subscribe to pwm signals from rc receiver
      self.rc_sub = rospy.Subscriber('/ackermann_cmd', AckermannDriveStamped, self.callback)

      # publish commands to vesc driver
      self.erpm_pub = rospy.Publisher('/commands/motor/speed', Float64, queue_size=2) 
      self.servo_pub = rospy.Publisher('/commands/servo/position', Float64, queue_size=2) 

   def callback(self, ackermann_msg):
      steer_rad = ackermann_msg.drive.steering_angle
      speed = ackermann_msg.drive.speed

      erpm = self.speed_to_erpm_gain * speed + self.speed_to_erpm_offset

      if abs(steer_rad) > self.rad_max:
         rospy.logwarn("Clipping steering command to +/- %.3f" %self.rad_max)
         steer_rad = max(min(steer_rad, self.rad_max), -self.rad_max) # clip [+/- steer_rad]

      if steer_rad > 0:
         val = self.rr_m * steer_rad + self.servo_mid
      else:
         val = self.lr_m * steer_rad + self.servo_mid
         
      self.servo_msg.data = max(min(val, self.servo_max), self.servo_min) # clip [0.0, 1.0]
      self.erpm_msg.data = erpm

      self.servo_pub.publish(self.servo_msg)
      self.erpm_pub.publish(self.erpm_msg)


   def load_params(self):
      self.wheelbase = rospy.get_param("/wheelbase")
      self.servo_mid = rospy.get_param("/servo_mid")
      self.servo_max = rospy.get_param("/servo_max")
      self.servo_min = rospy.get_param("/servo_min")
      self.lr_rmin = rospy.get_param("/lr_rmin")
      self.rr_rmin = rospy.get_param("/rr_rmin")
      self.speed_to_erpm_gain = rospy.get_param("/speed_to_erpm_gain")
      self.speed_to_erpm_offset = rospy.get_param("/speed_to_erpm_offset")
      

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

   # initialize node
   rospy.init_node('ackermann_to_vesc', anonymous=True)

   a2v = AckermannToVesc()

   try:
      rospy.spin()
   except KeyboardInterrupt:
      print("Shutting down ackermann_to_vesc")
