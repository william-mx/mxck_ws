#!/usr/bin/env python3

import rospy
import numpy as np
import cv2
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import time

class ImageSubscriber:
    def __init__(self):
        rospy.init_node('image_subscriber', anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/camera/color/image_jpeg', CompressedImage, self.image_callback)
        self.frame_count = 0
        self.start_time = time.time()

    def image_callback(self, msg):
        try:
            arr = np.frombuffer(msg.data, np.uint8)
            image = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            self.frame_count += 1

            if self.frame_count % 100 == 0:
                elapsed_time = time.time() - self.start_time
                self.start_time = time.time()
                mean_fps = 100 / elapsed_time
                rospy.loginfo("Mean FPS after {} frames: {:.2f}".format(self.frame_count, mean_fps))
        except Exception as e:
            rospy.logerr(e)

def main():
    image_subscriber = ImageSubscriber()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")

if __name__ == '__main__':
    main()
