#!/usr/bin/env python

import rospy
from std_msgs.msg import UInt16MultiArray
from sensor_msgs.msg import Joy
import numpy as np
import sys
import rospkg

def get_interp(x_vals, y_vals, thresholds = None):

   if thresholds is None:
      return lambda x: np.interp(x, x_vals, y_vals)
   
   x_expanded, y_expanded = [], []
   for x,y,t in zip(x_vals, y_vals, thresholds):
      if t:
         x = [x-t/2, x, x+t/2]
         y = [y, y, y]
      else:
         x, y = [x], [y]

      x_expanded.extend(x)
      y_expanded.extend(y)

      
   return lambda x: np.interp(x, x_expanded, y_expanded)


class RCJoystick:

   def __init__(self):
      
      # define thresholds
      self.mode_threshold = (100, 100, 100)
      self.steer_threshold = (0, 10, 0)
      self.speed_threshold = (0, 20, 0)
      
      # load parameters
      self.load_params()
      
      # define messages
      self.joy_msg = Joy()
      self.joy_msg.header.frame_id = 'rc_control'
      self.joy_msg.header.seq = 0
      self.joy_msg.axes = [0.0, 0.0] # init
      self.joy_msg.buttons = [0]
      
      # subscribe to pwm signals from rc receiver
      self.rc_sub = rospy.Subscriber('/veh_remote_ctrl', UInt16MultiArray, self.callback) # 20hz

      # publish joy message
      self.rc_pub = rospy.Publisher('/rc/joy', Joy, queue_size=1) 

   def parse_pwm(self, pwm_signal):

      steer_pwm, speed_pwm, mode_pwm = pwm_signal
      steer_val, speed_val, mode_val = 0, 0, 0 # init

      # connection lost if pwm = 0
      if 0 in pwm_signal:
         return steer_val, speed_val, mode_val
      

      steer_val = self.steer_mapping(steer_pwm)
      speed_val = self.speed_mapping(speed_pwm)
      mode_val = int(self.mode_mapping(mode_pwm))
         
      return steer_val, speed_val, mode_val

   def callback(self,data):

      steer_val, throt_val, mode_val = self.parse_pwm(data.data)
      
      self.joy_msg.axes[self.steer_ax] = steer_val
      self.joy_msg.axes[self.speed_ax] = throt_val
      
      self.joy_msg.buttons[self.mode_btn] = mode_val
      
      self.joy_msg.header.stamp = rospy.Time.now()
      self.joy_msg.header.seq += 1

      try:
         self.rc_pub.publish(self.joy_msg)
      except Exception as e:
         print(e)
         

   def load_params(self):
      
      # load parameters
      steer_min_pwm = rospy.get_param("/steering_min_pwm")
      steer_mid_pwm = rospy.get_param("/steering_mid_pwm")
      steer_max_pwm = rospy.get_param("/steering_max_pwm")

      speed_min_pwm = rospy.get_param("/speed_min_pwm")
      speed_mid_pwm = rospy.get_param("/speed_mid_pwm")
      speed_max_pwm = rospy.get_param("/speed_max_pwm")

      mode_min_pwm = rospy.get_param("/mode_min_pwm")
      mode_mid_pwm = rospy.get_param("/mode_mid_pwm")
      mode_max_pwm = rospy.get_param("/mode_max_pwm")

      self.steer_ax = rospy.get_param("/rc_steering_axis")
      self.speed_ax = rospy.get_param("/rc_speed_axis")
      self.mode_btn = rospy.get_param("/rc_mode_button")
      
      # add +/- threshold 
      self.steer_mapping = get_interp((steer_min_pwm, steer_mid_pwm, steer_max_pwm), (-1.0, 0.0, 1.0), self.steer_threshold)
      self.speed_mapping = get_interp((speed_min_pwm, speed_mid_pwm, speed_max_pwm), (-1.0, 0.0, 1.0), self.speed_threshold)
      self.mode_mapping = get_interp((mode_min_pwm, mode_mid_pwm, mode_max_pwm), (0, 1, 2), self.mode_threshold)

if __name__ == '__main__':

  # initialize node
  rospy.init_node('rc_joystick', anonymous=True)

  rcjoy = RCJoystick()

  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")