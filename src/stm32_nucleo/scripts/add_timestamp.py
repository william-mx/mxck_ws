#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray, Int16MultiArray
from stm32_nucleo.msg import Int16MultiArrayStamped

class stampedMsg:

   def __init__(self):


      # define messages
      self.imu_msg = Imu()
      self.imu_msg.header.frame_id = 'stm32_nucleo'
      self.imu_msg.header.seq = 0

      self.uss_msg = Int16MultiArrayStamped()
      self.uss_msg.header.frame_id = 'stm32_nucleo'
      self.uss_msg.header.seq = 0

      # subscribe to imu Float32MultiArray
      self.imu_sub = rospy.Subscriber('/imu', Float32MultiArray, self.imu_callback) # 200hz

      # subscribe to uss Int16MultiArray
      self.imu_sub = rospy.Subscriber('/uss_values', Int16MultiArray, self.uss_callback) # 50hz

      # publish as imu sensor_msg
      self.imu_pub = rospy.Publisher('/imu_stamped', Imu, queue_size=200)

      # publish as stamped std_msg
      self.uss_pub = rospy.Publisher('/uss_values_stamped', Int16MultiArrayStamped, queue_size=50)


   def imu_callback(self, msg):

      self.imu_msg.header.stamp = rospy.Time.now() # add timestamp
      self.imu_msg.header.seq += 1

      self.imu_msg.linear_acceleration.x = msg.data[0]
      self.imu_msg.linear_acceleration.y = msg.data[1]
      self.imu_msg.linear_acceleration.z = msg.data[2]
      self.imu_msg.angular_velocity.x = msg.data[3]
      self.imu_msg.angular_velocity.y = msg.data[4]
      self.imu_msg.angular_velocity.z = msg.data[5]

      self.imu_pub.publish(self.imu_msg)

   def uss_callback(self, msg):

      self.uss_msg.header.stamp = rospy.Time.now() # add timestamp
      self.uss_msg.header.seq += 1

      self.uss_msg.data = msg.data # copy data

      self.uss_pub.publish(self.uss_msg)

if __name__ == '__main__':

   # initialize node
   rospy.init_node('add_timestamps', anonymous=True)

   stamp = stampedMsg()

   try:
      rospy.spin()
   except KeyboardInterrupt:
      print("Shutting down")
