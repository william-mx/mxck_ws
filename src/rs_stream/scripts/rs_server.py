#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import pyrealsense2 as rs
import numpy as np
import cv2
import socketserver
import threading
import pickle
import struct
import signal

class RealSenseImagePublisher:
    def __init__(self, streaming_mode, width, height, fps, port):

        self.width = width
        self.height = height
        self.fps = fps
        self.port = port
        self.topic_name = '/camera/color/image_raw'
        
        streaming_modes = ['rs_stream', 'ros_stream']

        if not streaming_mode in streaming_modes:
            raise Exception(f"Unsupported streaming type. Choose between: {streaming_modes}")
        
        # Configure RealSense
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, self.width, self.height, rs.format.bgr8, self.fps)
        self.pipeline.start(config)
        
        if streaming_mode == 'rs_stream':
            rospy.loginfo("Streaming rgb image with %dx%d with %d FPS at port: %d" \
                %(self.width, self.height, self.fps, self.port))
            self.run_server()
        elif streaming_mode == 'ros_stream':
            rospy.loginfo("Streaming rgb image with %dx%d with %d FPS at %s" \
                %(self.width, self.height, self.fps, self.topic_name))
            self.image_pub = rospy.Publisher(self.topic_name, Image, queue_size=10)
            self.bridge = CvBridge()
            self.run()
            
    def run(self):
        try:
            while not rospy.is_shutdown():
                # Wait for the next set of frames from the camera
                frames = self.pipeline.wait_for_frames()

                # Extract color frame
                color_frame = frames.get_color_frame()

                if color_frame:
                    # Convert the RealSense color frame to a format compatible with cv2
                    color_image = np.asanyarray(color_frame.get_data())

                    # Convert the color image to a ROS Image message
                    ros_image_msg = self.bridge.cv2_to_imgmsg(color_image, encoding="bgr8")

                    # Set the timestamp of the ROS message
                    ros_image_msg.header.stamp = rospy.Time.now()

                    # Publish the ROS Image message
                    self.image_pub.publish(ros_image_msg)

        except rospy.ROSInterruptException:
            pass
        finally:
            # Stop the RealSense pipeline when the node is terminated
            self.pipeline.stop()

    def run_server(self):
        # Create an instance of the server listening on port 8888
        server = ThreadedRealsenseImageServer(('localhost', self.port), ThreadedRealsenseImageStreamHandler, self.pipeline)

        def signal_handler(signal, frame):
            print("Received Ctrl+C. Shutting down...")
            server.shutdown()
            server.server_close()
            print("Server shut down.")
            exit(0)
        
        signal.signal(signal.SIGINT, signal_handler)
        
        try:
            # Start the server in a separate thread
            server_thread = threading.Thread(target=server.serve_forever)
            server_thread.start()

            print(f"Server listening on {server.server_address}")

            # Wait for user interruption (Ctrl+C)
            server_thread.join()

        except KeyboardInterrupt:
            pass
        finally:
            # Close the RealSense pipeline
            self.pipeline.stop()
                    
class ThreadedRealsenseImageStreamHandler(socketserver.StreamRequestHandler):
    def handle(self):
        # Increase the count of connected clients
        self.server.client_count += 1
        print(f"Client {self.server.client_count} connected.")

        try:
            while True:
                # Send RealSense images to the client
                frames = self.server.get_frames()
                color_frame = frames.get_color_frame()

                if color_frame:
                    color_image = np.asanyarray(color_frame.get_data())
                    frame_data = pickle.dumps(color_image)
                    size = struct.pack('>Q', len(frame_data))
                    
                    # Send data to each connected client
                    for _ in range(self.server.client_count):
                        try:
                            self.request.sendall(size)
                            self.request.sendall(frame_data)
                        except BrokenPipeError:
                            print("Client disconnected.")
                            return

        except KeyboardInterrupt:
            print("Server interrupted by user.")
        finally:
            # Decrease the count of connected clients
            self.server.client_count -= 1
            print(f"Client {self.server.client_count + 1} disconnected.")

class ThreadedRealsenseImageServer(socketserver.ThreadingMixIn, socketserver.TCPServer):
    def __init__(self, server_address, handler_class, pipeline):
        super().__init__(server_address, handler_class)
        self.pipeline = pipeline
        self.client_count = 0  # Variable to count connected clients

    def get_frames(self):
        return self.pipeline.wait_for_frames()
    

if __name__ == "__main__":
    rospy.init_node('realsense_image_publisher', anonymous=True)
    
    fps = rospy.get_param("/rs_stream/fps", 30)
    width = rospy.get_param("/rs_stream/fps", 640)
    height = rospy.get_param("/rs_stream/fps", 360)
    port = rospy.get_param("/rs_stream/port", 9999)
    streaming_mode = rospy.get_param("/rs_stream/mode", 'rs_stream')

    valid_sizes = [(1920,1080), (1280,720), (960,540), (848,480), (640,480), \
                   (640,360), (424,240), (320,240), (320,180)]
    
    size = (width, height)
    
    if size not in valid_sizes:
        raise Exception(f"Unsupported image resolution; choose between: {valid_sizes}")
    
    if size == (1920,1080) or size == (1280,720):
        valid_fps = [6, 15, 30]
    else:
        valid_fps = [6, 15, 30, 60]
    
    if fps not in valid_fps:
        raise Exception(f"Unsupported image resolution; choose between: {valid_fps}")
    

    image_publisher = RealSenseImagePublisher(streaming_mode, width, height, fps, port)

