#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray

class toIMU:

   def __init__(self):


      # define messages
      self.imu_msg = Imu()
      self.imu_msg.header.frame_id = 'stm32_nucleo'
      self.imu_msg.header.seq = 0

      # subscribe to imu Float32MultiArray
      self.imu_sub = rospy.Subscriber('/imu', Float32MultiArray, self.callback) # 200hz

      # publish as imu sensor message
      self.imu_pub = rospy.Publisher('/imu_stamped', Imu, queue_size=200) 


   def callback(self, msg):

      self.imu_msg.header.stamp = rospy.Time.now()
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
   rospy.init_node('to_imu', anonymous=True)

   imu = toIMU()

   try:
      rospy.spin()
   except KeyboardInterrupt:
      print("Shutting down")
