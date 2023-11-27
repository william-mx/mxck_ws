#!/usr/bin/env python3

from cv_bridge import CvBridge, CvBridgeError

import cv2
import numpy as np
import rospkg
import rospy
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

def publish(im, detections):
  global id2color, bboxs, pub_markers, pub_image, bridge
  
  try:
    im_gray = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
    image_message = bridge.cv2_to_imgmsg(im_gray, encoding="mono8") # "bgr8"
  except CvBridgeError as e:
    print(e)
  
  pub_image.publish(image_message)
  
  bboxs.markers = []
  for d in detections:
    
    conf = d.Confidence
    id_ = d.ClassID
    
    points=[
      Point(d.Left, d.Top, 0),
      Point(d.Right, d.Top, 0),
      Point(d.Right, d.Bottom, 0),
      Point(d.Left, d.Bottom, 0),
    ]
    
    RGBA = id2color[id_] + (1,)

    bboxs.markers.append(
      ImageMarker(
        header=Header(stamp = rospy.Time.now()),
        scale=1,
        type=ImageMarker.POLYGON,
        outline_color=ColorRGBA(*RGBA),
        points=points,
      )
    )
  
  pub_markers.publish(bboxs)
        
if __name__ == '__main__':
  
  rospy.init_node('ssd_mobilenet', anonymous=True)     

  # Configure depth and color streams
  pipeline = rs.pipeline()
  config = rs.config()

  # Get device product line for setting a supporting resolution
  pipeline_wrapper = rs.pipeline_wrapper(pipeline)
  pipeline_profile = config.resolve(pipeline_wrapper)
  device = pipeline_profile.get_device()
  device_product_line = str(device.get_info(rs.camera_info.product_line))

  # check for rgb sensor
  found_rgb = False
  for s in device.sensors:
    if s.get_info(rs.camera_info.name) == 'RGB Camera':
      found_rgb = True
      break
  if not found_rgb:
    raise Exception("Realsense D435i RGB Camera Not found!")
    
  w, h, fps = 640, 480, 15
  config.enable_stream(rs.stream.color, w, h, rs.format.bgr8, fps)
          
  # build model
  r = rospkg.RosPack()
  model_dir = r.get_path('object_detection') + '/models/vdi_adc'
  model_onnx = model_dir + "/ssd-mobilenet.onnx"
  labels_txt = model_dir + "/labels.txt"
  
  for f in [model_dir, model_onnx, labels_txt]:
    if not os.path.exists(f):
      raise FileNotFoundError("No such file or directory: %s" %f)
  
  labels = open(labels_txt, 'r').read().splitlines()
  n_classes = len(labels)
  
  id2color = {
    0:  (1.0, 1.0, 1.0),
    1:  (0.0, 0.5, 1.0),
    2:  (1.0, 0.0, 1.0), 
    3:  (1.0, 0.0, 0.8), 
    4:  (0.0, 0.5, 0.8), 
    5:  (0.0, 1.0, 1.0),
    6:  (0.0, 0.8, 1.0),
    7:  (0.0, 1.0, 0.0),
    8:  (0.75, 0.75, 0.75),
    9:  (1.0, 0.0, 0.0),
    10: (1.0, 0.65, 0.0),
    11: (1.0, 1.0, 0.0)
  }
  
  
  id2label = {i: lbl for i, lbl in enumerate(labels)}
  print(id2label)
  
  threshold = 0.5 # confidence

  #build model
  net = detectNet("ssd-mobilenet-v2", model=model_onnx, labels=labels_txt, threshold=threshold, \
  input_blob='input_0', output_cvg='scores', output_bbox='boxes')
  
  # publish ros topics
  pub_image = rospy.Publisher("/camera/color/image_raw",Image, queue_size=1)
  pub_markers = rospy.Publisher( "/bboxs", ImageMarkerArray, queue_size=1)
  bboxs = ImageMarkerArray()
  bridge = CvBridge()

  # Start streaming
  pipeline.start(config)
  
  try:
    while not rospy.is_shutdown():

      # Wait for a coherent pair of frames: depth and color
      frames = pipeline.wait_for_frames()
      color_frame = frames.get_color_frame()
      if not color_frame:
        continue

      # Convert images to numpy arrays
      color_image = np.asanyarray(color_frame.get_data())

      cuda_img = cudaFromNumpy(color_image)
      detections = net.Detect(cuda_img)
      
      publish(color_image, detections)

  finally:

    # Stop streaming
    pipeline.stop()
        


