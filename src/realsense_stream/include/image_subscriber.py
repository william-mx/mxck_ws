import zmq
import cv2
import numpy as np
import threading
import rospy

class ImageSubscriber:
    def __init__(self, callback_fn, host="localhost", port=5555):
        self.active = True  # Control flag
        address = "tcp://{}:{}".format(host, port)
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.SUB)
        self.socket.connect(address)
        self.socket.setsockopt_string(zmq.SUBSCRIBE, u'')
        self.callback_fn = callback_fn
        
        print("Subscribing to {}".format(address))
        
        self.thread = threading.Thread(target=self.start_receiving)
        self.thread.daemon = True  # Set thread as daemon so it exits when the main program does
        self.thread.start()
        
    def start_receiving(self):
        try:
            while not rospy.is_shutdown():

                # First part: Timestamp
                timestamp_bytes = self.socket.recv()
                timestamp = float(timestamp_bytes.decode('utf-8'))
                
                # Second part: Image data
                frame = self.socket.recv()
                npimg = np.frombuffer(frame, dtype=np.uint8)
                image = cv2.imdecode(npimg, cv2.IMREAD_COLOR)
                self.callback_fn(image)
                
        finally:
            self.socket.close()
            self.context.term()
            print("Image subscriber successfully terminated. All resources have been cleaned up.")
