#!/usr/bin/env python3

from jetson_inference import detectNet
from jetson_utils import videoSource, videoOutput, cudaFromNumpy


import rospy
import cv2
import os
from collections import deque
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float32MultiArray
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

class DetectNet:          

  def __init__(self, DEBUG = False):
  
    
    self.id2label = {0: 'BACKGROUND',
                1:  'cross_parking',
                2:  'no_overtaking',
                3:  'overtaking',
                4:  'parallel_parking',
                5:  'pit_in',
                6:  'pit_out',
                7:  'traffic_light_green',
                8:  'traffic_light_off',
                9:  'traffic_light_red',
                10: 'traffic_light_red_yellow',
                11: 'traffic_light_yellow'} 

    self.num_classes = len(self.id2label) # incl. background
    self.N = 3 # mean confidence for N images
    self.mean_confidence = [0] * self.num_classes
    self.DEBUG = DEBUG
    
    self.conf_threshold = 0.5
    self.id2confidence = {i: deque([0]*self.N, maxlen=self.N) for i in self.id2label.keys()}
  
    # paths
    ws_dir = "/noetic_ws"
    #model_dir = ws_dir + "/jetson-inference/python/training/detection/ssd/models/custom_dataset"
    #model_dir = ws_dir + "/jetson-inference/python/training/detection/ssd/models/finetuning"
    model_dir = ws_dir + "/jetson-inference/python/training/detection/ssd/models/traffic_light"
    model_onnx = model_dir + "/ssd-mobilenet.onnx"
    labels_txt = model_dir + "/labels.txt"
    
    for f in [ws_dir, model_dir, model_onnx]:
      if not os.path.exists(f):
        raise FileNotFoundError("No such file or directory: %s" %f)
    
    self.bridge = CvBridge()
    
    #build model
    self.net = detectNet("ssd-mobilenet-v2", model=model_onnx, labels=labels_txt, threshold=0.5, \
    input_blob='input_0', output_cvg='scores', output_bbox='boxes')
    
    # subscribe to camera message
    self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.callback)
    
    # publish class
    self.pub = rospy.Publisher('/prediction', String, queue_size=1)
    self.bbox_pub = rospy.Publisher('/boundingbox', Float32MultiArray, queue_size=1)

    self.bbox_msg = Float32MultiArray()
    
  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
      cuda_img = cudaFromNumpy(cv_image)
      #rospy.loginfo("Image recieved")
      detections = self.net.Detect(cuda_img)
      
      confs = [0] * self.num_classes
      
      bbox_data = []

      for d in detections:
        id_ = d.ClassID
        label = self.id2label[id_]
        confidence = round(d.Confidence, 2)
        start_point = (int(d.Left), int(d.Top))
        end_point = (int(d.Right), int(d.Bottom))
        cv_image = cv2.rectangle(cv_image, start_point, end_point, (255,0,0), 2)
        confs[id_] = confidence

        if (confidence>=0.5):
          bbox_data.extend([id_, d.Left, d.Top, d.Right, d.Bottom])

        if self.DEBUG:
          org = (int(d.Left), int(sorted([0, d.Top - 10, cv_image.shape[0]])[1]))
          cv_image = cv2.putText(cv_image, "%s: %d%%" % (label, confidence*1e2), org, cv2.FONT_ITALIC, 1, (255, 255, 255), 2, cv2.LINE_AA)

      self.bbox_msg.data = bbox_data

      self.bbox_pub.publish(self.bbox_msg)
        
      for i in range(self.num_classes):
        self.id2confidence[i].append(confs[i])
        self.mean_confidence[i] = np.mean(self.id2confidence[i])
      
      if np.max(self.mean_confidence) >= self.conf_threshold:
        id_ = np.argmax(self.mean_confidence)
        lbl = self.id2label[id_]
        self.pub.publish(lbl)
        rospy.loginfo("%s detected: %s" % (lbl, str(self.id2confidence[id_])))
      
      if self.DEBUG:
        cv2.imshow("Image window", cv_image)
        cv2.waitKey(200) # ms
    except CvBridgeError as e:
      print(e)

    
    
if __name__ == '__main__':
  rospy.init_node('detectnet', anonymous=True)     

  sim = DetectNet(DEBUG=False)          

  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
    
  cv2.destroyAllWindows()

