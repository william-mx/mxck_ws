#!/usr/bin/env python3
import rospy
import cv2
import sys
import os
import rospkg
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from dynamic_reconfigure.server import Server
from dynamic_reconfigure.client import Client
from lane_detection.cfg import advanced_lane_detectionConfig
import matplotlib.pyplot as plt
import time


class LaneDetection:

  def __init__(self, DEBUG = False):
    
    self.list_imgmsgs = []
    self.list_run = []
    self.DEBUG = DEBUG # visualize if DEBUG
    
    self.img_configured = False  # Flag indicating whether image parameters (like width) have been saved
    self.config_received = False  # Flag indicating whether configuration has been received from rqt dynamic reconfigure
    self.config_parsed = False  # Flag indicating whether parameters have been successfully converted
    self.lane_configured = False  # Flag indicating whether features related to the lane (like width) have been saved

    self.max_out = 3 # Vehicle will stopp if no lane line detected for max_out frames
    self.LEFT, self.RIGHT = 0, 1 # Enum
    self.center_list = np.empty((0,2), dtype = int) # Line center coodinates [[LEFT, RIGHT]]
    self.success_list = np.empty((0,2), dtype = bool) # Flag if line detected [[True, False]]
    
    self.state = 'unknown' # Initialize state ['unknown', 'staright', 'left_curve', 'right_curve', 'lost']
    
    self.ideal_pixels = None # Number of pixels if line is  perfectly detected
    self.min_pixels = None # Mimimum pixels to count detection as valid
    self.lane_width = None # Lane width in pixels
    
    # Represents the offset from the vehicle center to the lane center.
    # The offset is normalized to the image width, ensuring compatibility across different resolutions.
    # For example, if the offset is 0.2, the corresponding offset in pixels would be 0.2 * image_width.
    self.offset_msg = Float32()

    # Instantiate a CvBridge object for converting between ROS Image messages and OpenCV images.
    self.bridge = CvBridge() 

    # Publishes the offset from the center of the detected lane.
    self.pub_offset = rospy.Publisher('/offset', Float32, queue_size=1)

    # Publishes images with overlaid markers for debugging and visualization purposes.
    # self.pub_viz = rospy.Publisher("/lane_visualization", Image, queue_size=1)

    # Subscribes to the camera message from the realsense2_camera node.
    self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.callback)

    # Lane parameters
    self.params_set = False  # Flag indicating whether lane detection parameters have been set

    # Launches a dynamic reconfigure server for updating lane detection parameters.
    self.srv = Server(advanced_lane_detectionConfig, self.update_params)
    self.client = Client('/advanced_lane_detection', timeout=30)   

    while not rospy.is_shutdown():
      pass
    
    if self.DEBUG:
      #save plot
      r = rospkg.RosPack()
      directory = r.get_path('lane_detection') + '/results'
      self.plot_values(directory)
      
      cv2.destroyAllWindows()
      
  # ---------------------------------
  # State Machine
  # ---------------------------------

  def run(self, image):
    
    # Save image shape only once during the first callback
    if not self.img_configured:
      self.set_image_params(image)
        
    # Skip further processing if dynamic reconfigure parameters are not set
    if not self.config_recieved:
      return
    elif not self.config_parsed:
      # Preprocess configuration
      self.parse_config()
    
    self.im_preprocessed = self.preprocess_image(image)
    
    self.im_out = cv2.cvtColor(self.im_preprocessed, cv2.COLOR_GRAY2RGB)
    
    if self.state == 'unknown':
      success_left, center_left = self.sliding_window(image, self.LEFT, center = self.win_xleft)
      success_right, center_right = self.sliding_window(image, self.RIGHT, color = (0,0,255), center = self.win_xright)
      
      self.success_list = np.vstack([self.success_list, [success_left, success_right]])
      self.center_list = np.vstack([self.center_list, [center_left, center_right]])

      self.calculate_offset(image)
      
    elif self.state == 'straight':
      success_left, center_left = self.sliding_window(image, self.LEFT)
      success_right, center_right = self.sliding_window(image, self.RIGHT, color = (0,0,255))

      self.success_list = np.vstack([self.success_list, [success_left, success_right]])
      self.center_list = np.vstack([self.center_list, [center_left, center_right]])
      
      self.calculate_offset(image)
      
      self.update_state()
      
    elif self.state == 'left_curve':
      
      success_left, center_left = False, self.center_list[-1,0]
      
      success_right, center_right = self.sliding_window(image, self.RIGHT, color = (0,0,255))
      
      if success_right:
        im_center_left = center_right - self.lane_width # imaginary center
        if im_center_left > self.limit_left:
          success_left, center_left = self.sliding_window(image, self.LEFT, center = im_center_left)
          
      self.success_list = np.vstack([self.success_list, [success_left, success_right]])
      self.center_list = np.vstack([self.center_list, [center_left, center_right]])

      self.calculate_offset(image)
      
      self.update_state()
      
    elif self.state == 'right_curve':

      success_right, center_right = False, self.center_list[-1,0]
      
      success_left, center_left = self.sliding_window(image, self.LEFT)
      
      if success_left:
        im_center_right = center_left + self.lane_width # imaginary center
        if im_center_right < self.limit_right:
          success_right, center_right = self.sliding_window(image, self.RIGHT, center = im_center_right)
          
      self.success_list = np.vstack([self.success_list, [success_left, success_right]])
      self.center_list = np.vstack([self.center_list, [center_left, center_right]])

      self.calculate_offset(image)
      
      self.update_state()
      
    
    if self.DEBUG:
      cv2.imshow('Image', image)
      cv2.waitKey(1)
  

  def update_state(self):
    
    if len(self.success_list) < self.max_out: return 
    
    success_left = not np.all(self.success_list[-self.max_out:, 0] == False)
    success_right = not np.all(self.success_list[-self.max_out:, 1] == False)
    
    if success_left and success_right:
      self.state = 'straight'
    elif success_left:
      self.state ='right_curve'
    elif success_right:
      self.state = 'left_curve'
    else:
      self.state = 'straight' # start searching for lines

  
  # ---------------------------------
  # Preprocessing
  # ---------------------------------

  def preprocess_image(self, image):

      # Convert to grayscale
      self.im_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

      # Apply Gaussian blur to the grayscale image
      im_blur = cv2.GaussianBlur(self.im_gray, self.kernel, 0)

      # Apply Canny edge detection to the preprocessed image
      im_preprocessed = cv2.Canny(im_blur, self.canny_min, self.canny_max) # [0 or 255]
      
      return im_preprocessed
    
  # ---------------------------------
  # Configuration
  # ---------------------------------

  def update_params(self, config, level):

      self.config_parsed = False
      
      self.canny_min = config['canny_min'] # Minimum threshold for Canny edge detection.
      self.canny_max = config['canny_max'] # Maximum threshold for Canny edge detection.
      
      # kerner must be an odd number
      k_ = config['k']
      k = k_ if k_ % 2 != 0 else k_ - 1
      self.kernel = (k, k) # Define the kernel size for Gaussian blur (width, height)
      
      self.win_h_ = config['win_h'] # windows height [%]
      self.win_w_ = config['win_w'] # windows width [%]
      self.win_xleft_ = config['win_xleft'] # left window center in x [%]
      self.win_xright_ = config['win_xright'] # right window center in x [%]
      self.win_y_ = config['win_y'] # windows center in y [%]
      self.num_windows = config['num_windows'] # number of windows
      self.setup_done = config['setup_done'] # programm startes when setup is done

      self.config_recieved = True

      return config
    
  def parse_config(self):
    
    # convert from [0.0,...,1.0] to [pixels]
    self.win_h = int(self.win_h_ * self.im_h) # windows height [px]
    self.win_w = int(self.win_w_ * self.im_w) # windows width [px]
    self.win_xleft = int(self.win_xleft_ * self.im_w) # left window center in x [px]
    self.win_xright = int(self.win_xright_ * self.im_w) # right window center in x [px]
    self.win_y = int(self.win_y_ * self.im_h) # windows center in y [px]

    rospy.loginfo(
        f"canny_min: {self.canny_min}, canny_max: {self.canny_max}, "
        f"kernel: {self.kernel}, "
        f"win_h: {self.win_h}, win_w: {self.win_w}, "
        f"win_xleft: {self.win_xleft}, win_xright: {self.win_xright}, win_y: {self.win_y}, "
        f"num_windows: {self.num_windows}, "
        f"setup_done: {self.setup_done}"
    )
    
    if self.setup_done:
      success = self.set_lane_params()
      if success: self.state = 'straight'
        
    self.config_parsed = True
        
  def set_image_params(self, image):
    (self.im_h, self.im_w, _) = image.shape
    self.limit_left = int(0.03 * self.im_w)
    self.limit_right = int(0.97 * self.im_w)
    self.cam_center = self.im_w // 2
    self.img_configured = True
    
    rospy.loginfo(f"{self.im_h=} {self.im_w=} {self.limit_left=} {self.limit_right=}")

  def set_lane_params(self):
    success_left, success_right = self.success_list[-1]
    
    if success_left and success_right:
      center_left, center_right = self.center_list[-1]
      self.lane_width = int(center_right - center_left)
      self.cam_center = (center_right + center_left) // 2
      self.ideal_pixels = self.num_pixels_
      self.min_pixels = int(0.2 * self.num_pixels_)
      
      rospy.loginfo(f"{self.lane_width=} {self.cam_center=} {self.ideal_pixels=} {self.min_pixels=}")
      
      self.lane_configured = True
      
      return True
      
    else:
      rospy.logwarn("Either the left or right lane line is not detected correctly.")

      # Reset the parameter to its default value
      self.client.update_configuration({'setup_done': False})

      return False
            
  # ---------------------------------
  # Detection
  # ---------------------------------

  def sliding_window(self, image, index, color = (0, 255, 0), center = None):

    self.path = []
    center = center or self.center_list[-1,index]

    sucess, updated_center, next_center = self.find_center(image, center, self.win_y, color)
    
    if not sucess:
      return False, updated_center
    
    cy = self.win_y
    for _ in range(self.num_windows - 1):
      cy -=  self.win_h
      sucess, _, next_center = self.find_center(image, next_center, cy, color)

      
      if not sucess:
        break
        
    return True, updated_center
  
  def find_center(self, image, cx, cy, color):

    # Calculate box coordinates and clip values
    x1 = np.maximum(0, int(cx - self.win_w / 2))
    y1 = np.maximum(0, int(cy - self.win_h / 2))
    x2 = np.minimum(self.im_w, x1 + self.win_w)
    y2 = np.minimum(self.im_h, y1 + self.win_h)
    
    if self.DEBUG:
      self.draw_box(image, x1, y1, x2, y2, color)
      
    patch = self.im_preprocessed[y1:y2, x1:x2]
    
    _, xs = np.nonzero(patch)
    
    # no line detected
    if len(xs) == 0:
      return False, cx, None
    
    self.num_pixels_ = len(xs)
    
    updated_center = int(x1 + np.mean(xs))
    self.path += [updated_center]
    
    # Check if the updated lane center is too close to the image border
    if updated_center < self.limit_left or updated_center > self.limit_right:
        return False, updated_center, None

      
    if self.num_windows < 2:
      return True, updated_center, None
    
    if len(self.path) >= 2:
      dx = self.path[-1] - self.path[-2] # dx = x2 - x1
    else:
      #mean_x_per_row = np.apply_along_axis(lambda r: np.mean(np.nonzero(r)), axis=1, arr=patch)
      #dx = mean_x_per_row[0] - mean_x_per_row[-1]
      values = np.array([(np.mean(np.nonzero(row)), self.win_h - y) for y, row in enumerate(patch) if np.count_nonzero(row) > 0]) # (N,2)
      dx = values[0,0] - values[-1,-0]

    next_center = updated_center + dx
    
    return True, updated_center, next_center


  # ---------------------------------
  # Controlling
  # ---------------------------------
  
  def calculate_offset(self, image):

    success_left, success_right = self.success_list[-1]
    center_left, center_right = self.center_list[-1]
    
    if self.lane_width is None: # remove
      self.lane_width = center_right - center_left
      
    if success_left and success_right:
      self.veh_center = (center_left + center_right) // 2
    elif success_left:
      self.veh_center = center_left + self.lane_width // 2
    elif success_right:
      self.veh_center = center_right - self.lane_width // 2

    # Calculate and normalize the offset in pixels relative to the image position
    self.offset = (self.cam_center - self.veh_center) / self.im_w  # Positive for offset (+) turn left, negative for offset (-) turn right
    
    if self.config_parsed:
      self.offset_msg.data = self.offset
      self.pub_offset.publish(self.offset_msg)
    
    if self.DEBUG:
      self.plot_position(image)
    

  # ---------------------------------
  # Visualization
  # ---------------------------------
  
  def draw_vertical_line(self, image, x, y, color = (0, 255, 255), thickness = 1, length = 10):
    cv2.line(image, (x, y - length), (x, y + length), color, thickness)

  def draw_horizontal_line(self, image, x_start, x_end, y, color = (0, 255, 255), thickness = 1):
    cv2.line(image, (x_start, y), (x_end, y), color, thickness)
  
  def draw_horizontal_arrow(self, image, x_start, x_end, y, color = (0, 0, 255), thickness = 1, arrow_length=50):
    # Draw a vertical line at the end of the arrow
    self.draw_vertical_line(image, x_start, y, color, thickness)
  
    # Draw the horizontal arrow
    arrow_length *= np.sign(x_start - x_end)
    cv2.arrowedLine(image, (int(x_start + arrow_length), y), (x_start, y), color, thickness, tipLength=0.3)
    
    
  def draw_box(self, image, x1, y1, x2, y2, color=(0, 255, 0), thickness = 2):

    # Draw the box on the image
    cv2.rectangle(image, (x1, y1), (x2, y2), color=color, thickness=thickness)

    # Extract the patch from the grayscale image
    patch = self.im_out[y1:y2, x1:x2, :]

    # Replace the corresponding region in the RGB image with the patch
    image[y1:y2, x1:x2, :] = patch

    return image

  def plot_position(self, image):
    success_left, success_right = self.success_list[-1]
    center_left, center_right = self.center_list[-1]
    
    x1 = center_left if success_left else 0
    x2 = center_right if success_right else self.im_w
  
    self.draw_horizontal_line(image, x1, x2, self.win_y)
    
    for ctr, suc in zip([center_left, center_right], [success_left, success_right]):
      if suc:
        self.draw_vertical_line(image, ctr, self.win_y)
    
    
    self.draw_vertical_line(image, self.veh_center, self.win_y)
    
    self.draw_horizontal_arrow(image, self.cam_center, self.veh_center, self.win_y -35)
    
    cv2.putText(image, f'Offset: {self.offset:.2f}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
    
  def plot_values(self, directory):
    
    fpath = directory + '/offset_right_px.png'
    plt.scatter(np.where(self.success_list[:, 1])[0], self.center_list[self.success_list[:, 1], 1])
    plt.title('Horizontal Position of the Right Line in Image [Pixels]')
    plt.savefig(fpath)
    print("Saved to %s" %fpath)
    
    fpath = directory + '/offset_left_px.png'
    plt.scatter(np.where(self.success_list[:, 0])[0], self.center_list[self.success_list[:, 0], 1])
    plt.title('Horizontal Position of the Left Line in Image [Pixels]')
    plt.savefig(fpath)
    print("Saved to %s" %fpath)
      

  def callback(self, data):
          
    try:
      image = self.bridge.imgmsg_to_cv2(data, "bgr8")
      self.run(image)
    except CvBridgeError as e:
      print(e)
        
    


if __name__ == '__main__':
  rospy.init_node('lane_detection', anonymous=True)

  ld = LaneDetection(DEBUG = True)
