#!/usr/bin/env python

import rospy
from std_msgs.msg import Int16MultiArray
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
import rospkg
import numpy as np
import os

class PDCvisualization:

  def __init__(self):

    # base directory 
    r = rospkg.RosPack()
    base_dir = r.get_path('pdc_visualization')

    # load pdc visualization image
    img_fpath = base_dir + '/images/pdc_visualization_template.tiff'

    if not os.path.exists(img_fpath):
        rospy.logerr("Image is not available at %s.", img_fpath)

    self.pdc_template = cv2.imread(img_fpath)

    # store shape
    height, width, _ = self.pdc_template.shape
    self.size = (width,height)

    # load pixel coordinates for each patch; shape (num_sensors x num_sections)
    coor_fpath = base_dir + '/images/patch_px_coords.pkl'

    if not os.path.exists(coor_fpath):
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
    self.uss_sub = rospy.Subscriber('/uss_values', Int16MultiArray, self.callback, queue_size=1)

    # publish PDC visualization
    self.pdc_pub = rospy.Publisher("pdc_visualization",CompressedImage, queue_size=1)
    
    # info message n sensors detected 
    self.num_detected = None

    # Create CompressedImage message
    self.jpeg_msg = CompressedImage()
    self.jpeg_msg.format = "jpeg"
    
  def visualize_pdc(self, data):
    
    # convert to numpy array
    measurement = np.array(data.data).reshape(-1, 1)
    
    # print status message
    num = np.count_nonzero(measurement != -2)
    if num != self.num_detected:
        rospy.loginfo("%d/%d sensors detected" %(num, self.num_sonars))
        self.num_detected = num
        
    measurement[measurement < 0] = self.sonar_max # remove (-1) invalid measurement, (-2) no sensor detected
    

    # check shape
    if measurement.shape[0] != self.num_sonars:
        rospy.logerr("Received %s measurements, expected %s.", measurement.shape[0], self.num_sonars)

    pdc_image = self.pdc_template.copy()

    measurement_mat = np.repeat(measurement, self.num_sections, axis = 1)

    result = (self.sections_mat <= measurement_mat)
    
    try:
        gray_patches = np.concatenate(self.patch_px_coords[result], axis = 1)

        indices = (gray_patches[0], gray_patches[1])

        pdc_image[indices] = self.gray_color
    except ValueError:
        pass # all sensors on min

    return pdc_image


  def callback(self,data):
    pdc_image = self.visualize_pdc(data)
    
    try:
        
        # Encode image as JPEG
        _, img_encoded = cv2.imencode('.jpg', pdc_image)

        self.jpeg_msg.header.stamp = rospy.Time.now()
        self.jpeg_msg.data = np.array(img_encoded).tobytes()

        self.pdc_pub.publish(self.jpeg_msg)

    except Exception as e:
        print(e)

if __name__ == '__main__':

  # initialize node
  rospy.init_node('pdc_visualization', anonymous=True)

  pdc = PDCvisualization()

  while not rospy.is_shutdown():
    pass

    

