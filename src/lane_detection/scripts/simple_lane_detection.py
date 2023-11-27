#!/usr/bin/env python3
import rospy
import cv2
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from ackermann_msgs.msg import AckermannDriveStamped
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from dynamic_reconfigure.server import Server
from lane_detection.cfg import lane_detectionConfig

class SimpleLaneDetection:

  def __init__(self):

    self.bridge = CvBridge()
    
    # define messages
    self.offMsg = Float32()

    # subscribe to camera message
    self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.callback)

    # publish offset from center
    self.offset_pub = rospy.Publisher('/offset', Float32, queue_size=1)
    
    # lane parameters
    self.lane_w = None
    
    self.parameters_set = False

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
    

  def callback(self,data):

    if not self.parameters_set: return
    
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)


    ''' detect lane lines '''

    (im_h, im_w, im_c) = cv_image.shape
    im_cx = im_w // 2

    # convert to grayscale
    im_gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

    # blur image
    im_blur = cv2.GaussianBlur(im_gray, (5, 5), 0)

    # apply canny edge detection
    im_canny = cv2.Canny(im_blur, self.canny_min, self.canny_max)

    # crop image
    ymin_ = int(im_h * self.ymin)
    ymax_ = int(im_h * self.ymax)
    ymid = (ymax_ + ymin_) // 2
    im_croped = im_canny[ymin_:ymax_, ...]

    # to binary
    im_binary = np.where(im_croped > 0, 1, 0)

    ''' visualize '''

    # highlight the cropped area
    im_out = np.vstack((cv_image[:ymin_,...],
                        np.dstack((im_canny, im_canny, im_canny))[ymin_:ymax_,...],
                        cv_image[ymax_:,...]))
    
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
      cv2.circle(im_out, (x_left, ymid), 8, (255,0,0), -1)
      
    if max_r > self.peak_min:
      index = self.n_bins//2 + np.argmax(hist[self.n_bins//2 :])
      x_right = (index + 1) * w_bin + w_bin // 2
      cv2.circle(im_out, (x_right, ymid), 8, (255,0,0), -1)
    
    if max_r > self.peak_min and max_l > self.peak_min:
      self.lane_w = x_right - x_left # width
      lane_cx = x_left + self.lane_w // 2 # center
      offset = im_cx - lane_cx # offset, offset left (-), offset right (+)
    elif max_r > self.peak_min and not self.lane_w is None:
      offset = im_cx - (x_right -  self.lane_w // 2)
    elif max_l > self.peak_min and not self.lane_w is None:
      offset = im_cx - (x_left + self.lane_w // 2)
    
    try:
      self.offMsg.data = offset
      self.offset_pub.publish(self.offMsg)
    except:
      pass


    




                      
    # plot histogram
    plotx = np.repeat(bins, 2)[1:-1]
    ploty = np.repeat(hist, 2)
    line = np.asarray(tuple(zip(plotx, ymax_ - ploty)), np.int32)
    cv2.polylines(im_out, [line], False, (0,0,255), thickness = 2)
    

    
    # show result
    cv2.imshow("Result", im_out)
    cv2.waitKey(3)

if __name__ == '__main__':
  rospy.init_node('lane_detection', anonymous=True)

  sim = SimpleLaneDetection()

  srv = Server(lane_detectionConfig, sim.update_params)
  
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
    
  cv2.destroyAllWindows()
