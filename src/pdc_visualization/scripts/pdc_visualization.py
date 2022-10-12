#!/usr/bin/env python

import rospy
from std_msgs.msg import Int16MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import rospkg
import numpy as np
import os
from datetime import datetime

class pdc_visualization:

  def __init__(self):

    # output paramters
    self.to_video = False # save result as video
    self.publish = False # publish result as image message
    self.live = True # show result live

    # base directory 
    r = rospkg.RosPack()
    base_dir = r.get_path('pdc_visualization')

    # load pdc visualization image
    img_fpath = base_dir + '/images/pdc_visualization_template.tiff'

    if not os.path.exists(img_fpath):
        rospy.logerr("Image is not available at %s.", img_fpath)

    self.pdc_template = cv2.imread(img_fpath)

    # store shape
    height, width, layers = self.pdc_template.shape
    self.size = (width,height)

    # save result as video
    results_dir = base_dir + '/results'
    
    result_fname = results_dir + "/result_{:%Y_%m_%d_%H_%M_%S}.avi".format(datetime.now())
    self.fps = 50 # check frquency with rostopic hz /uss_values
    self.out = cv2.VideoWriter(result_fname,cv2.VideoWriter_fourcc(*'MJPG'), self.fps, self.size)

    # load pixel coordinates for each patch; shape (num_sensors x num_sections)
    coor_fpath = base_dir + '/images/patch_px_coords.pkl'

    if not os.path.exists(img_fpath):
        rospy.logerr("Numpy pixel coordinates not available at %s.", coor_fpath)

    self.patch_px_coords = np.load(coor_fpath, allow_pickle=True, fix_imports=True)

    # Define sections
    # If the measured value is in the section, the section is colored.

    self.sonar_min = 2.0 # cm
    self.sonar_max = 100.0 # cm
    self.num_sections = 6
    self.num_sonars = 10

    # the further away the measurement is, the higher the measurement uncertainty
    # assumed variance: [2/18, 2/18, 3/18, 3/18, 4/18, 4/18]

    sections = np.array(self.sonar_max * np.array([2./18, 2./18, 3./18, 3./18, 4./18]), dtype=np.uint8)
    sections = np.add.accumulate(sections)
    sections = np.concatenate([sections, np.array([self.sonar_max])]).reshape(1, self.num_sections)
    self.sections_mat = np.repeat(sections, self.num_sonars, axis = 0)

    # Inactive Section get colored im gray color
    self.gray_color = (245,245,245)

    self.bridge = CvBridge()

    # subscribe to distance measurement from ultrasonic sensors
    self.uss_sub = rospy.Subscriber('/uss_values', Int16MultiArray, self.callback)

    # publish PDC visualization
    self.pdc_pub = rospy.Publisher("pdc_visualization",Image, queue_size=5)

  def visualize_pdc(self, data):

    # convert to numpy array
    measurement = np.array(data.data).reshape(-1, 1)

    measurement[measurement == -1] = self.sonar_max

    # check shape
    if measurement.shape[0] != self.num_sonars:
        rospy.logerr("Received %s measurements, expected %s.", measurement.shape[0], self.num_sonars)

    pdc_image = self.pdc_template.copy()

    measurement_mat = np.repeat(measurement, self.num_sections, axis = 1)

    result = (self.sections_mat <= measurement_mat)

    gray_patches = np.concatenate(self.patch_px_coords[result], axis = 1)

    indices = (gray_patches[0], gray_patches[1])

    pdc_image[indices] = self.gray_color

    return pdc_image

  def ctrl_shutdown(self):
    '''controlled shutdown'''

    # close video
    if pdc.to_video:
      pdc.out.release()
      

  def callback(self,data):

    pdc_image = self.visualize_pdc(data)

    # save result as video
    if self.to_video:
      self.out.write(pdc_image)

    # publish as image message
    if self.publish:
      try:
        pass
        self.image_message = self.bridge.cv2_to_imgmsg(pdc_image, encoding="bgr8")
      except CvBridgeError as e:
        print(e)
      
      self.pdc_pub.publish(self.image_message)
    
    # show result live 
    if self.live:
      cv2.imshow("Image window", pdc_image)
      cv2.waitKey(100) # ms

    


if __name__ == '__main__':

  # initialize node
  rospy.init_node('pdc_visualization', anonymous=True)

  pdc = pdc_visualization()

  try:
    rospy.spin()
  except KeyboardInterrupt:
    pdc.ctrl_shutdown()
    

