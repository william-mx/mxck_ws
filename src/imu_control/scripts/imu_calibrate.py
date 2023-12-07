#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray, Int16MultiArray
from scipy.signal import butter, filtfilt
from collections import deque



          
class IMUcalib:

   def __init__(self, apply_filter = True):
      
      # define messages
      self.imu_msg = Imu()
      self.imu_msg.header.frame_id = 'stm32_nucleo'
      self.imu_msg.header.seq = 0

      # calibrated
      self.n_samples = 1600
      self.is_calibrated = False
      self.data = []
      self.mean_values = None
      self.std_values = None
      self.target_values = np.array([0.0, 0.0, 9.81, 0.0, 0.0, 0.0, 0.0]) # accel_x, accel_y, accel_z, ang_x, ang_y, ang_z, temperature
      self.correction = []
      
      # https://www.samproell.io/posts/yarppg/yarppg-live-digital-filter/
      # filter measurements
      # init butterworth filter
      
      self.apply_filter = apply_filter
      
      fs = 200.0 # sample rate hz
      nyq = fs/2 # nyquist frequency
      cutoff_frequency = 2.0 # cutoff frequency in hz
      norm_freq = cutoff_frequency / nyq # normalized from 0 to 1
      order = 2
      
      # numerator (b) and denominator (a) polynomials of the IIR filter.
      b, a = butter(order, norm_freq, btype='low', analog=False)
      
      self.ms = np.zeros(shape = (len(b), len(self.target_values))) # measurements (3,7)
      self.fm = np.zeros(shape = (len(a) - 1, len(self.target_values))) # filtered measurements (2,7)
      
      self.b, self.a = np.expand_dims(b, axis=0), np.expand_dims(a, axis=0) # (3,) -> (1,3)
      
      # info message
      rospy.loginfo("calibrating imu with %d samples" %self.n_samples)
      rospy.logwarn("do not move the carkit during the calibration process")
      
      # subscribe to imu Float32MultiArray
      self.imu_sub = rospy.Subscriber('/imu', Float32MultiArray, self.imu_callback) # 200hz

      # publish as imu sensor_msg
      self.imu_pub = rospy.Publisher('/imu_calibrated', Imu, queue_size=200)
      self.filter_pub = rospy.Publisher('/imu_filtered', Imu, queue_size=200)
      
   def calibrate(self, msg):
      self.data += [msg.data]
      
      if len(self.data) == self.n_samples:
         self.data = np.array(self.data)
         self.mean_values = np.mean(self.data, axis = 0)
         self.std_values = np.std(self.data, axis = 0)
         self.correction = self.target_values - self.mean_values
         self.correction[-1] = 0.0 # no correction on temerature
         
         # info message
         rospy.loginfo("calibration finished")
         rospy.loginfo("mean: accel_x: %.2f, accel_y: %.2f, accel_z: %.2f, ang_x: %.2f, ang_y: %.2f, ang_z: %.2f, temperature: %.2f" %tuple(self.mean_values))
         rospy.loginfo("std: accel_x: %.2f, accel_y: %.2f, accel_z: %.2f, ang_x: %.2f, ang_y: %.2f, ang_z: %.2f, temperature: %.2f" %tuple(self.std_values))
         
         if self.apply_filter:
           rospy.loginfo("applying butterworth filter")
         
   def butterworth_filter(self, data):
      # Filter incoming data with standard difference equations.

      self.ms = np.concatenate(([data], self.ms), axis = 0)
      self.ms = np.delete(self.ms, (-1), axis = 0)

      y = np.dot(self.b, self.ms) - np.dot(self.a[:,1:], self.fm)
      y = y / self.a[:,0]

      self.fm = np.concatenate((y, self.fm), axis = 0)
      self.fm = np.delete(self.fm, (-1), axis = 0)
      
      return y[0]
      
      
   def imu_callback(self, msg):
      
      if len(self.data) < self.n_samples:
         self.calibrate(msg)
         return
      
      # apply correction
      data = np.array(msg.data) + self.correction 

      self.imu_msg.header.stamp = rospy.Time.now() # add timestamp
      self.imu_msg.header.seq += 1
      
      # publish calibrated imu data
      self.imu_msg.linear_acceleration.x = data[0]
      self.imu_msg.linear_acceleration.y = data[1]
      self.imu_msg.linear_acceleration.z = data[2]
      self.imu_msg.angular_velocity.x = data[3]
      self.imu_msg.angular_velocity.y = data[4]
      self.imu_msg.angular_velocity.z = data[5]

      self.imu_pub.publish(self.imu_msg)
      
      
      # apply mean filter
      if self.apply_filter:
        data = self.butterworth_filter(data)

        # publish filtered imu data
        self.imu_msg.linear_acceleration.x = data[0]
        self.imu_msg.linear_acceleration.y = data[1]
        self.imu_msg.linear_acceleration.z = data[2]
        self.imu_msg.angular_velocity.x = data[3]
        self.imu_msg.angular_velocity.y = data[4]
        self.imu_msg.angular_velocity.z = data[5]

        self.filter_pub.publish(self.imu_msg)
      


if __name__ == '__main__':

   # initialize node
   rospy.init_node('calibrate_imu', anonymous=True)

   stamp = IMUcalib(apply_filter = True)

   try:
      rospy.spin()
   except KeyboardInterrupt:
      print("Shutting down")
