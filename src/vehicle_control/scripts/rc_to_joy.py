#!/usr/bin/env python

import rospy
from std_msgs.msg import UInt16MultiArray
from sensor_msgs.msg import Joy

class RCJoystick:

   def __init__(self):

      # load parameters
      self.load_params()

      # define thresholds to filter out noise
      self.mode_pwm_threshold = 100
      self.throt_pwm_threshold = 12
      self.steer_pwm_threshold = 5

      # define messages
      self.joy_msg = Joy()
      self.joy_msg.header.frame_id = 'rc_control'
      self.joy_msg.header.seq = 0

      # subscribe to pwm signals from rc receiver
      self.rc_sub = rospy.Subscriber('/veh_remote_ctrl', UInt16MultiArray, self.callback) # 20hz

      # publish joy message
      self.rc_pub = rospy.Publisher('/rc/joy', Joy, queue_size=1) 

   def parse_pwm(self, pwm_signal):

      steer_pwm, throt_pwm, mode_pwm = pwm_signal
      steer_val, throt_val, mode_val = 0, 0, 0 # init

      # connection lost if pwm = 0
      if 0 in pwm_signal:
         return steer_val, throt_val, mode_val
      
      # steering
      if abs(steer_pwm - self.steer_mid_pwm) < self.steer_pwm_threshold:
         steer_val = 0.0
      elif steer_pwm >= self.steer_mid_pwm:
         steer_val = min(float(steer_pwm - self.steer_mid_pwm) / (self.steer_max_pwm - self.steer_mid_pwm), 1.0) # [0.0, 1.0]
      else:
         steer_val = max(float(steer_pwm - self.steer_mid_pwm) / (self.steer_mid_pwm - self.steer_min_pwm), -1.0) # [-1.0, 0.0]

         
      # throttle
      if abs(throt_pwm - self.throt_mid_pwm) < self.throt_pwm_threshold:
         throt_val = 0.0
      elif throt_pwm >= self.throt_mid_pwm:
         throt_val = min(float(throt_pwm - self.throt_mid_pwm) / (self.throt_max_pwm - self.throt_mid_pwm), 1.0) # [0.0, 1.0]
      else:
         throt_val = max(float(throt_pwm - self.throt_mid_pwm) / (self.throt_mid_pwm - self.throt_min_pwm), -1.0) # [-1.0, 0.0]
      
      # mode
      if abs(self.mode_min_pwm - mode_pwm) < self.mode_pwm_threshold:
         mode_val = 0
      elif abs(self.mode_mid_pwm - mode_pwm) < self.mode_pwm_threshold:
         mode_val =  1
      elif abs(self.mode_max_pwm - mode_pwm) < self.mode_pwm_threshold:
         mode_val =  2
      
      return steer_val, throt_val, mode_val


   def callback(self,data):

      steer_val, throt_val, mode_val = self.parse_pwm(data.data)

      self.joy_msg.axes = [steer_val, throt_val]
      self.joy_msg.buttons = [mode_val]
      self.joy_msg.header.stamp = rospy.Time.now()
      self.joy_msg.header.seq += 1

      try:
         self.rc_pub.publish(self.joy_msg)
      except Exception as e:
         print(e)
         

   def load_params(self):
      
      # load parameters
      self.steer_min_pwm = rospy.get_param("/steering_min_pwm")
      self.steer_mid_pwm = rospy.get_param("/steering_mid_pwm")
      self.steer_max_pwm = rospy.get_param("/steering_max_pwm")

      self.throt_min_pwm = rospy.get_param("/throttle_min_pwm")
      self.throt_mid_pwm = rospy.get_param("/throttle_mid_pwm")
      self.throt_max_pwm = rospy.get_param("/throttle_max_pwm")

      self.mode_min_pwm = rospy.get_param("/mode_min_pwm")
      self.mode_mid_pwm = rospy.get_param("/mode_mid_pwm")
      self.mode_max_pwm = rospy.get_param("/mode_max_pwm")

if __name__ == '__main__':

  # initialize node
  rospy.init_node('rc_joystick', anonymous=True)

  rcjoy = RCJoystick()

  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")