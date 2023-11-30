#!/usr/bin/env python3
import rospy
import cv2
import sys
import os
import rospkg
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from ackermann_msgs.msg import AckermannDriveStamped
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from dynamic_reconfigure.server import Server
from lane_detection.cfg import lane_detectionConfig

# add the file path of the package so that interpeter can find the module
r = rospkg.RosPack()
sys.path.append(os.path.join(r.get_path('lane_detection'), 'include'))

from rs_stream import RGBstream

class SimpleLaneDetection:

  def __init__(self, rs_stream = False, live = False):

    # message offset from center in pixels
    self.offMsg = Float32()

    self.bridge = CvBridge()
    
    # live visualization
    self.live = live

    # publish offset from center
    self.pub_offset = rospy.Publisher('/offset', Float32, queue_size=1)
    
    # publish images with overlaid markers for debugging purposes
    self.pub_viz = rospy.Publisher("/lane_visualization",Image, queue_size=1)
    
    # lane parameters
    self.lane_w = 685 # lane width in pixels 
    self.parameters_set = False
    
    # launch dynamic reconfig server
    self.srv = Server(lane_detectionConfig, self.update_params)

    if rs_stream:
      # stream images using pyrealsense2
      self.cam_stream = RGBstream(width=1280, height=720, fps=15)
      self.start_streaming()
    else:
      # subscribe to camera message using realsense2_camera node 
      self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.callback)
      
    
  def update_params(self, config, level):
      self.ymin = config['ymin']
      self.ymax = config['ymax']
      self.canny_min = config['canny_min']
      self.canny_max = config['canny_max']
      self.n_bins = config['n_bins']
      self.peak_min = config['peak_min']
      self.parameters_set = True

      rospy.loginfo("ymin: {ymin}, ymin: {ymin}, canny_min: {canny_min}, canny_max: {canny_max}, n_bins: {n_bins}, peak_min: {peak_min}""".format(**config))
      
      return config
  
  def publish_viz(self, image, f = 0.5):

    try:
      # convert the image to grayscale for reduced data transfer
      im_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
      
      # make image smaller for reduced data transfer
      w, h = int(im_gray.shape[1] * f), int(im_gray.shape[0] * f)
      im_small = cv2.resize(im_gray, (w, h))
      
      # to ros message
      image_message = self.bridge.cv2_to_imgmsg(im_small, encoding="mono8") # "bgr8"
      
      # publish
      self.pub_viz.publish(image_message)
      
    except CvBridgeError as e:
      print(e)
  
  
    
  def run_detection(self, image):

    ''' preprocess image '''

    (im_h, im_w, _) = image.shape
    im_cx = im_w // 2 # center

    # convert to grayscale
    im_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # blur image
    im_blur = cv2.GaussianBlur(im_gray, (9, 9), 0)

    # apply canny edge detection
    im_canny = cv2.Canny(im_blur, self.canny_min, self.canny_max)

    # crop image
    ymin_ = int(im_h * self.ymin)
    ymax_ = int(im_h * self.ymax)
    ymid = (ymax_ + ymin_) // 2
    im_croped = im_canny[ymin_:ymax_, ...]

    # to binary
    im_binary = np.where(im_croped > 0, 1, 0)


    ''' search for lines '''
    
    # calculate histogram
    bins = np.linspace(start = 0, stop = im_w, num = self.n_bins).astype(int)
    hist = [im_binary[:, x1:x2].sum() for (x1, x2) in zip(bins[:-1], bins[1:])]
    
    # get peaks
    w_bin = im_w // self.n_bins # bin width in pixels
    max_l = np.max(hist[: self.n_bins//2])
    max_r = np.max(hist[self.n_bins//2 :])
    
    if max_l > self.peak_min:
      index = np.argmax(hist[: self.n_bins//2])
      x_left = index * w_bin + w_bin // 2
      
    if max_r > self.peak_min:
      index = self.n_bins//2 + np.argmax(hist[self.n_bins//2 :])
      x_right = (index + 1) * w_bin + w_bin // 2

    
    
    ''' Visualize Results '''
    
    # highlight the cropped area
    im_out = np.vstack((image[:ymin_,...],
                        np.dstack((im_canny, im_canny, im_canny))[ymin_:ymax_,...],
                        image[ymax_:,...]))
    
    # plot histogram
    plotx = np.repeat(bins, 2)[1:-1]
    ploty = np.repeat(hist, 2)
    line = np.asarray(tuple(zip(plotx, ymax_ - ploty)), np.int32)
    cv2.polylines(im_out, [line], False, (0,0,255), thickness = 2)
    
    # plot peaks as arrow
    if max_l > self.peak_min:
      cv2.arrowedLine(im_out, (x_left, ymid - 50), (x_left, ymid), (0,255,255), thickness = 3, tipLength = 0.5)
    if max_r > self.peak_min:
      cv2.arrowedLine(im_out, (x_right, ymid - 50), (x_right, ymid), (255,0,255), thickness = 3, tipLength = 0.5)
    
    self.publish_viz(im_out)
    
    if self.live:
      cv2.imshow("Result", im_out)
      cv2.waitKey(3) # milliseconds 

    ''' Caluculate offset '''
       
    if max_r <= self.peak_min and max_l <= self.peak_min:
      return # no lines detected
    
    if max_r > self.peak_min and max_l > self.peak_min:
      lane_cx = x_left + self.lane_w // 2 # center
    elif max_r > self.peak_min and not self.lane_w is None:
      lane_cx = x_right -  self.lane_w // 2
    elif max_l > self.peak_min and not self.lane_w is None:
      lane_cx = x_left + self.lane_w // 2
    
    offset = im_cx - lane_cx # offset, offset left (-), offset right (+)
    
    try:
      self.offMsg.data = offset
      self.pub_offset.publish(self.offMsg)
    except Exception as es:
      print(e)

  
  def start_streaming(self):

    try:
      while not rospy.is_shutdown():
        
        if not self.parameters_set: continue
        
        # Wait for RGB image
        image = self.cam_stream.wait_for_image()
        self.run_detection(image)

    finally:

      # Stop streaming
      self.cam_stream.stop()
    
  def callback(self, data):
    if not self.parameters_set: return
    
    try:
      image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    
    self.run_detection(image)


if __name__ == '__main__':
  rospy.init_node('lane_detection', anonymous=True)

  sim = SimpleLaneDetection(rs_stream = False, live = False)

  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  
  if sim.live:
    cv2.destroyAllWindows()
