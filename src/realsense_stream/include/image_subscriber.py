import zmq
import cv2
import numpy as np

class ImageSubscriber:
    def __init__(self, host="localhost", port=5555):

        self.address = f"tcp://{host}:{port}"
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.SUB)
        self.socket.connect(self.address)
        self.socket.setsockopt_string(zmq.SUBSCRIBE, '')

        print("Subscribing to {}".format(self.address))
        
    def wait_for_frame(self):
        try:
            # First part: Timestamp
            timestamp_bytes = self.socket.recv()
            timestamp = float(timestamp_bytes.decode('utf-8'))
            
            # Second part: Image data
            frame = self.socket.recv()
            npimg = np.frombuffer(frame, dtype=np.uint8)
            image = cv2.imdecode(npimg, cv2.IMREAD_COLOR)


            return timestamp, image
        except zmq.ZMQError as e:
            print(f"ZMQ error occurred: {e}")
        except Exception as e:
            print(f"An error occurred: {e}")

    def close(self):
        self.socket.close()
        self.context.term()

    def __del__(self):
        self.close()

    # Context manager support
    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()
