import socket
import pickle
import struct
import cv2
import numpy as np

class ImageReceiver:
    def __init__(self, host='localhost', port=9999):
        self.host = host
        self.port = port
        self.client_socket = None

    def connect_to_server(self):
        # Create a socket and connect to the server
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client_socket.connect((self.host, self.port))

    def receive_frames(self):
        data = b''  # Initialize data buffer

        try:
            while True:
                # Receive the size of the data
                size_data = self.client_socket.recv(8)
                if not size_data:
                    break

                size = struct.unpack('>Q', size_data)[0]

                # Receive the actual data
                while len(data) < size:
                    packet = self.client_socket.recv(size - len(data))
                    if not packet:
                        break
                    data += packet

                # Deserialize the data to a numpy array
                frame = pickle.loads(data)

                # Display the received image using OpenCV
                # cv2.imshow('Received Image', frame)
                # cv2.waitKey(1)  # Adjust the delay as needed for display responsiveness

                data = b''  # Reset data buffer

        except KeyboardInterrupt:
            pass
        finally:
            # Close the socket
            self.client_socket.close()
            # cv2.destroyAllWindows()

if __name__ == "__main__":
    receiver = ImageReceiver()
    receiver.connect_to_server()
    receiver.receive_frames()
