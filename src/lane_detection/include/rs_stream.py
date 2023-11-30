import pyrealsense2 as rs
import numpy as np

class RGBstream:
  
  def __init__(self, width=640, height=480, fps=15):
    # Configure depth and color streams
    self.pipeline = rs.pipeline()
    self.config = rs.config()

    # Get device product line for setting a supporting resolution
    pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
    pipeline_profile = self.config.resolve(pipeline_wrapper)
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

    self.config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
    self.pipeline.start(self.config)

  def wait_for_image(self):
    # Wait for a coherent pair of frames: depth and color
    frames = self.pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()
    
    # Convert images to numpy arrays
    color_image = np.asanyarray(color_frame.get_data())
      
    return color_image
      
  def stop(self):
    self.pipeline.stop()
