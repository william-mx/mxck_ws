#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan, Image
import cv2
import numpy as np
import matplotlib.pyplot as plt
from cv_bridge import CvBridge, CvBridgeError



class ScanVisualization:

  def __init__(self):

    self.bridge = CvBridge()

    # subscribe to distance measurement from ultrasonic sensors
    self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.callback)

    # publish scan visualization
    self.scan_pub = rospy.Publisher("scan_visualization", Image, queue_size=1)

  def callback(self,data):
    # put your code here
    pass

    
if __name__ == '__main__':

  # initialize node
  rospy.init_node('scan_visualization', anonymous=True)

  sviz = ScanVisualization()

  while not rospy.is_shutdown():
    pass
    

