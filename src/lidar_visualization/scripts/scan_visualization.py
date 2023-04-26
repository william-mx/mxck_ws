#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan, Image
import numpy as np
import matplotlib.pyplot as plt
from cv_bridge import CvBridge, CvBridgeError
from  matplotlib.colors import LinearSegmentedColormap


class ScanVisualization:

  def __init__(self):

    self.bridge = CvBridge()

    # subscribe to lidar measurement
    self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.callback, queue_size=1, buff_size = 2**24)

    # publish scan visualization
    self.scan_pub = rospy.Publisher("scan_visualization", Image, queue_size=1)

    # custom colorbar
    self.cmap = LinearSegmentedColormap.from_list('rg',["red", "orange", "green"], N=256) 

  def callback(self,data):

    r = np.array(data.ranges) # ranges
    theta = np.linspace(data.angle_min, data.angle_max, len(r)) # angles

    fig, ax = plt.subplots(subplot_kw={'projection': 'polar'})
    pts = ax.scatter(theta, r, c = r, cmap = self.cmap, marker = '.')
    ax.set_rticks([]) # ticks off
    ax.set_rlim(0, r[np.isfinite(r)].max() + .1)

    fig.colorbar(pts, orientation = 'horizontal')

    # draw figure
    fig.canvas.draw()

    # get the image as RGB
    scan_image = np.frombuffer(fig.canvas.tostring_rgb(), dtype=np.uint8)

    # add color channel and reshape
    scan_image = scan_image.reshape(fig.canvas.get_width_height()[::-1] + (3,))

    # close fig
    plt.close()

    # convert to ros image
    try:
      self.image_msg = self.bridge.cv2_to_imgmsg(scan_image, encoding="rgb8")
    except CvBridgeError as e:
      print(e)
    
    # publish
    self.scan_pub.publish(self.image_msg)

    
if __name__ == '__main__':

  # initialize node
  rospy.init_node('scan_visualization', anonymous=True)

  sviz = ScanVisualization()

  while not rospy.is_shutdown():
    pass
    

