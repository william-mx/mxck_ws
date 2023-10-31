#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray, Int16MultiArray

class IMUcalib:

   def __init__(self):


      # define messages
      self.imu_msg = Imu()
      self.imu_msg.header.frame_id = 'stm32_nucleo'
      self.imu_msg.header.seq = 0

      # subscribe to imu Float32MultiArray
      self.imu_sub = rospy.Subscriber('/imu', Float32MultiArray, self.imu_callback) # 200hz

      # publish as imu sensor_msg
      self.imu_pub = rospy.Publisher('/imu_filtered', Imu, queue_size=200)

      # calibrated
      self.n_samples = 1200
      self.is_calibrated = False
      self.data = []
   
   def calibrate(self, msg):
      values = np.array(msg.data)
      print(values.shape)
   
   def imu_callback(self, msg):
      
      self.calibrate(msg)
      

      self.imu_msg.header.stamp = rospy.Time.now() # add timestamp
      self.imu_msg.header.seq += 1

      self.imu_msg.linear_acceleration.x = msg.data[0]
      self.imu_msg.linear_acceleration.y = msg.data[1]
      self.imu_msg.linear_acceleration.z = msg.data[2]
      self.imu_msg.angular_velocity.x = msg.data[3]
      self.imu_msg.angular_velocity.y = msg.data[4]
      self.imu_msg.angular_velocity.z = msg.data[5]

      self.imu_pub.publish(self.imu_msg)



if __name__ == '__main__':

   # initialize node
   rospy.init_node('calibrate_imu', anonymous=True)

   stamp = IMUcalib()

   try:
      rospy.spin()
   except KeyboardInterrupt:
      print("Shutting down")
