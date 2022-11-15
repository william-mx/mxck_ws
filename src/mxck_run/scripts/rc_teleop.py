#!/usr/bin/env python

import rospy
from std_msgs.msg import UInt16MultiArray
from ackermann_msgs.msg import AckermannDriveStamped

# load parameters
steering_min_pwm = rospy.get_param("/steering_min_pwm")
steering_mid_pwm = rospy.get_param("/steering_mid_pwm")
steering_max_pwm = rospy.get_param("/steering_max_pwm")

throttle_min_pwm = rospy.get_param("/throttle_min_pwm")
throttle_mid_pwm = rospy.get_param("/throttle_mid_pwm")
throttle_max_pwm = rospy.get_param("/throttle_max_pwm")

mode_min_pwm = rospy.get_param("/mode_min_pwm")
mode_mid_pwm = rospy.get_param("/mode_mid_pwm")
mode_max_pwm = rospy.get_param("/mode_max_pwm")

servo_min = rospy.get_param("/servo_min")
servo_mid = rospy.get_param("/servo_mid")
servo_max = rospy.get_param("/servo_max")

speed_min = rospy.get_param("/speed_min")
speed_mid = rospy.get_param("/speed_mid")
speed_max = rospy.get_param("/speed_max")

mode_mid = 0

class RcControl:

  def __init__(self):

    # define thresholds to filter out noise
    self.mode_pwm_threshold = 100
    self.throttle_pwm_threshold = 12
    self.steering_pwm_threshold = 5

    # define messages
    self.ackMsg = AckermannDriveStamped()

    # subscribe to pwm signals from rc receiver
    self.rc_sub = rospy.Subscriber('/veh_remote_ctrl', UInt16MultiArray, self.callback)

    # publish ackermann messages to VESC
    self.ackermann_pub = rospy.Publisher('/ackermann_cmd', AckermannDriveStamped, queue_size=10) 

  def parse_pwm(self, pwm_signal):
    steering_pwm, throttle_pwm, mode_pwm = pwm_signal

    # connection lost if pwm = 0
    if 0 in pwm_signal:
       return servo_mid, speed_mid, mode_mid
    
    # steering
    if abs(steering_pwm - steering_mid_pwm) < self.steering_pwm_threshold:
       steering_ack = 0.0
    elif steering_pwm <= steering_mid_pwm:
       steering_ack = servo_min * (steering_pwm - steering_mid_pwm) / (steering_min_pwm - steering_mid_pwm)
    else:
       steering_ack = servo_max * (steering_pwm - steering_mid_pwm) / (steering_max_pwm - steering_mid_pwm)

       
    # throttle
    if abs(throttle_pwm - throttle_mid_pwm) < self.throttle_pwm_threshold:
       throttle_ack = 0.0
    elif throttle_pwm >= throttle_mid_pwm:
       throttle_ack = speed_min * (throttle_pwm - throttle_mid_pwm) / (throttle_min_pwm - throttle_mid_pwm)
    else:
       throttle_ack = speed_max * (throttle_pwm - throttle_mid_pwm) / (throttle_max_pwm - throttle_mid_pwm)
	 

    # mode
    if abs(mode_min_pwm - mode_pwm) < self.mode_pwm_threshold:
       mode = -1
    elif abs(mode_mid_pwm - mode_pwm) < self.mode_pwm_threshold:
       mode =  0
    elif abs(mode_max_pwm - mode_pwm) < self.mode_pwm_threshold:
       mode =  1
    
    return steering_ack, throttle_ack, mode


  def callback(self,data):

    steering_ack, throttle_ack, mode = self.parse_pwm(data.data)

    self.ackMsg.header.stamp = rospy.Time.now()
    self.ackMsg.drive.steering_angle = steering_ack
    self.ackMsg.drive.speed = throttle_ack

    self.ackermann_pub.publish(self.ackMsg)


if __name__ == '__main__':

  # initialize node
  rospy.init_node('rc_control', anonymous=True)

  rc = RcControl()

  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
