#!/usr/bin/env python3
import socket
import pickle
import struct

from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import rospkg
import rospy
import sys
import os
from jetson_inference import detectNet
from jetson_utils import cudaFromNumpy
from sensor_msgs.msg import Image
from std_msgs.msg import ColorRGBA, Header
from foxglove_msgs.msg import ImageMarkerArray
from visualization_msgs.msg import ImageMarker
from geometry_msgs.msg import Point
import matplotlib.pyplot as plt
import pyrealsense2 as rs
   
r = rospkg.RosPack()
sys.path.append(os.path.join(r.get_path('rs_stream'), 'include'))

from client import ImageReceiver
          
class SSDMobileNet:
    def __init__(self, mode = 'ros_stream', port=9999):
      
        rospy.init_node('ssd_mobilenet', anonymous=True)

        self.bridge = CvBridge()

        # Build model
        self.build_model()

        # ID to color mapping
        self.id2color = {
            0: (1.0, 1.0, 1.0),
            1: (0.0, 0.5, 1.0),
            2: (1.0, 0.0, 1.0),
            3: (1.0, 0.0, 0.8),
            4: (0.0, 0.5, 0.8),
            5: (0.0, 1.0, 1.0),
            6: (0.0, 0.8, 1.0),
            7: (0.0, 1.0, 0.0),
            8: (0.75, 0.75, 0.75),
            9: (1.0, 0.0, 0.0),
            10: (1.0, 0.65, 0.0),
            11: (1.0, 1.0, 0.0)
        }

        if mode == 'ros_stream':
          # Subscribe to camera message using realsense2_camera node
          self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.callback)
        elif mode == 'rs_stream':
          receiver = ImageReceiver('localhost', port, self.run_detection)
          receiver.receive_frames()

        while not rospy.is_shutdown():
            pass

    def build_model(self):
        r = rospkg.RosPack()
        model_dir = r.get_path('object_detection') + '/models/vdi_adc'
        model_onnx = model_dir + "/ssd-mobilenet.onnx"
        labels_txt = model_dir + "/labels.txt"

        for f in [model_dir, model_onnx, labels_txt]:
            if not os.path.exists(f):
                raise FileNotFoundError("No such file or directory: %s" % f)

        labels = open(labels_txt, 'r').read().splitlines()

        self.n_classes = len(labels)
        self.id2label = {i: lbl for i, lbl in enumerate(labels)}

        threshold = 0.5  # Confidence

        # Build model
        self.net = detectNet("ssd-mobilenet-v2", model=model_onnx, labels=labels_txt, threshold=threshold,
                             input_blob='input_0', output_cvg='scores', output_bbox='boxes')

        # Publish ROS topics
        self.pub_image = rospy.Publisher("/camera/gray/image_raw", Image, queue_size=1)
        self.pub_markers = rospy.Publisher("/bboxs", ImageMarkerArray, queue_size=1)
        self.bboxs = ImageMarkerArray()

    def publish(self, im, detections):
        try:
            im_gray = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
            image_message = self.bridge.cv2_to_imgmsg(im_gray, encoding="mono8")  # "bgr8"
        except CvBridgeError as e:
            print(e)

        self.pub_image.publish(image_message)

        self.bboxs.markers = []
        for d in detections:
            conf = d.Confidence
            id_ = d.ClassID

            points = [
                Point(d.Left, d.Top, 0),
                Point(d.Right, d.Top, 0),
                Point(d.Right, d.Bottom, 0),
                Point(d.Left, d.Bottom, 0),
            ]

            RGBA = self.id2color[id_] + (1,)

            self.bboxs.markers.append(
                ImageMarker(
                    header=Header(stamp=rospy.Time.now()),
                    scale=1,
                    type=ImageMarker.POLYGON,
                    outline_color=ColorRGBA(*RGBA),
                    points=points,
                )
            )

        self.pub_markers.publish(self.bboxs)

    def run_detection(self, image):
        cuda_img = cudaFromNumpy(image)
        detections = self.net.Detect(cuda_img)

        self.publish(image, detections)
        
    def callback(self, data):
        try:
            image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        self.run_detection(image)

            
if __name__ == '__main__':
    port = rospy.get_param("/rs_server/port", 9999)
    mode = rospy.get_param("/rs_server/mode", 'ros_stream')
    

    
    ssd_mobilenet = SSDMobileNet(mode, port)
