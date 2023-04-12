#!/usr/bin/env python
import rospy
import cv2
from sensor_msgs.msg import Image
from ackermann_msgs.msg import AckermannDriveStamped
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

class SimpleLaneDetection:

  def __init__(self):

    self.bridge = CvBridge()
    
    # define messages
    self.ackMsg = AckermannDriveStamped()

    # subscribe to camera message
    self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.callback)

    # publish ackermann messages to VESC
    self.ackermann_pub = rospy.Publisher('/ackermann_cmd', AckermannDriveStamped, queue_size=1) 

  def callback(self,data):

    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)


    ''' detect lane lines '''

    (im_h, im_w, im_c) = cv_image.shape

    # convert to grayscale
    im_gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

    # blur image
    im_blur = cv2.GaussianBlur(im_gray, (5, 5), 0)

    # apply canny edge detection
    im_canny = cv2.Canny(im_blur, 80, 240)

    # crop image
    ymin = int(im_h * 0.75)
    ymax = int(im_h * 0.80)
    im_croped = im_canny[ymin:ymax, ...]

    # to binary
    im_binary = np.where(im_croped > 0, 1, 0)

    # calculate histogram
    bins = np.linspace(start = 0, stop = im_w, num = 16).astype(int)
    hist = [im_binary[:, x1:x2].sum() for (x1, x2) in zip(bins[:-1], bins[1:])]

    ''' send dummy control commands '''

    self.ackMsg.header.stamp = rospy.Time.now()
    self.ackMsg.drive.steering_angle = 0
    self.ackMsg.drive.speed = 0

    self.ackermann_pub.publish(self.ackMsg)

    ''' visualize '''

    # highlight the cropped area
    im_out = np.vstack((cv_image[:ymin,...],
                        np.dstack((im_canny, im_canny, im_canny))[ymin:ymax,...],
                        cv_image[ymax:,...]))
                      
    # plot histogram
    plotx = np.repeat(bins, 2)[1:-1]
    ploty = np.repeat(hist, 2)
    line = np.asarray(tuple(zip(plotx, ymax - ploty)), np.int32)
    cv2.polylines(im_out, [line], False, (0,0,255), thickness = 2)

    # show result
    cv2.imshow("Result", im_out)
    cv2.waitKey(3)

if __name__ == '__main__':
  rospy.init_node('lane_detection', anonymous=True)

  sim = SimpleLaneDetection()

  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
    
  cv2.destroyAllWindows()