#!/usr/bin/env python

import rospy
import yaml
from std_msgs.msg import UInt16MultiArray
import rospkg
import numpy as np
import signal
import threading
from collections import OrderedDict
import json

class CalibrationWizard:
    def __init__(self, fpath):
        # Set up the signal handler for Ctrl+C
        signal.signal(signal.SIGINT, self.signal_handler)
        
        # read control config
        self.read_config(fpath)
        

        # Configuration for calibration
        hz = 40  # /veh_remote_ctrl ends with 40hz
        n_seconds = 2  # stay for n_seconds in each position
        n_values = hz * n_seconds
        self.recieved_pwm = np.zeros(shape=(n_values, 3))
        self.output = np.zeros((3, 3), dtype=np.bool_)

        self.shutdown_requested = False
        self.lock = threading.Lock()
        self.status = 0
        
        # subscribe to pwm signal from rc
        rospy.Subscriber('/veh_remote_ctrl', UInt16MultiArray, self.callback)
        
    def signal_handler(self, sig, frame):
        # Handle Ctrl+C signal
        with self.lock:
            self.shutdown_requested = True

    def await_calibration_completion(self):
        # Wait for calibration completion
        rate = rospy.Rate(1)

        while not rospy.is_shutdown():
            with self.lock:
                if self.shutdown_requested:
                    rospy.logerr("The calibration has failed")
                    return False
                elif bool(self.output.prod()):
                    self.clr_print("The calibration has been successful", 'green')
                    return True
            rate.sleep()

    def read_config(self, file_path='config.yaml'):
        # Read configuration from YAML file
        try:
            with open(file_path, 'r') as file:
                config_data = yaml.safe_load(file)
        except Exception as e:
            print("Error reading YAML file: {}".format(e))
            return

        # Set default values for mean and std thresholds
        self.mean_threshold = np.array([
            [7.0, 3.0, 7.0],
            [7.0, 3.0, 7.0],
            [7.0, 3.0, 7.0],
        ])
        self.std_threshold = np.array([
            [0.6],
            [0.6],
            [0.6],
        ])

        # Create an ordered dictionary for configuration keys
        keys = ['steering_min_pwm',
                'steering_mid_pwm',
                'steering_max_pwm',
                'throttle_min_pwm',
                'throttle_mid_pwm',
                'throttle_max_pwm',
                'mode_min_pwm',
                'mode_mid_pwm',
                'mode_max_pwm']

        self.config = OrderedDict()
        for k in keys:
            self.config[k] = config_data.get(k, 0)
        self.mat = np.array(list(self.config.values())).reshape(3, 3)

        print(json.dumps(self.config, indent=4))

    def clr_print(self, text, color):
        # Print colored text to the console
        colors = {
            'reset': '\033[0m',
            'black': '\033[30m',
            'red': '\033[91m',
            'green': '\033[92m',
            'yellow': '\033[93m',
            'blue': '\033[94m',
            'purple': '\033[95m',
            'cyan': '\033[96m',
            'white': '\033[97m'
        }

        if color in colors:
            print("{}{}{}".format(colors[color], text, colors['reset']))
        else:
            print(text)

    def callback(self, data):
        
        # Callback function for the subscriber
        pwm_signal = np.array(data.data).reshape(1, 3)

        # Queue for received PWM signals
        self.recieved_pwm = np.concatenate((self.recieved_pwm, pwm_signal), axis=0)
        self.recieved_pwm = np.delete(self.recieved_pwm, (0), axis=0)

        # Calculate mean and std of received PWM signals
        mean = np.mean(self.recieved_pwm, axis=0).reshape(3, 1)
        std = np.std(self.recieved_pwm, axis=0).reshape(3, 1)
            
        # Check if mean and std are within thresholds
        tmp_mean = abs(self.mat - mean) < self.mean_threshold
        tmp_std = std < self.std_threshold
        result = tmp_mean * tmp_std
        self.output[result] = True

        # Print status if it has changed
        if self.output.sum() != self.status:
            msg = "steering: %s throttle: %s mode: %s" % (
                str(self.output[0]), str(self.output[1]), str(self.output[2]))
            self.clr_print(msg, 'yellow')
            self.status = self.output.sum()

if __name__ == '__main__':
    rospy.init_node('rc_safety_check', anonymous=True)
    
    # Main execution when the script is run
    r = rospkg.RosPack()
    fpath = r.get_path('vehicle_control') + '/config/control_config.yaml'

    # Create RemoteControlListener instance
    wiz = CalibrationWizard(fpath)
    success = wiz.await_calibration_completion()

