#!/usr/bin/env python

import os
import rospy
import rospkg
from std_msgs.msg import UInt16MultiArray
from lighting_control import LightingControl


# import python modules from different ROS packages
import sys

r = rospkg.RosPack()
sys.path.append(os.path.join(r.get_path('synced_record'), 'scripts'))

from synced_record import SyncedRecord


# load parameters
steering_min_pwm = rospy.get_param("/steering_min_pwm")
steering_mid_pwm = rospy.get_param("/steering_mid_pwm")
steering_max_pwm = rospy.get_param("/steering_max_pwm")

throttle_min_pwm = rospy.get_param("/throttle_min_pwm")
throttle_mid_pwm = rospy.get_param("/throttle_mid_pwm")
throttle_max_pwm = rospy.get_param("/throttle_max_pwm")

mode_min_pwm = rospy.get_param("/mode_min_pwm")
mode_mid_pwm = rospy.get_param("/mode_mid_pwm")
mode_max_pwm = rospy.get_param("/mode_max_pwm")

class rc_commands:

    def __init__(self):

        # define rough thresholds to filter out noise
        self.mode_pwm_threshold = 100
        self.throttle_pwm_threshold = 50
        self.steering_pwm_threshold = 50

        # subscribe to pwm signals from rc receiver
        self.rc_sub = rospy.Subscriber('/veh_remote_ctrl', UInt16MultiArray, self.callback)

        # define modes
        self.prev_mod = None # previous mode
        self.curr_mod = None
        self.prev_smod = None # previous steering mode
        self.curr_smod = None
        self.prev_tmod = None # throttle steering mode
        self.curr_tmod = None

        self.mod_changed = False
        self.smod_changed = False
        self.tmod_changed = False

        self.is_connected = True
	
        # topics
        self.lit = LightingControl()
        self.sr = SyncedRecord()
        

    def parse_pwm(self, pwm_signal):

        steering_pwm, throttle_pwm, mode_pwm = pwm_signal

        # connection lost if pwm = 0
        if 0 in pwm_signal:
            self.is_connected = False
            return
        else:
            self.is_connected = True


        # steering
        if abs(steering_pwm - steering_mid_pwm) < self.steering_pwm_threshold:
            smod =  0 # straight
        elif steering_pwm <= steering_mid_pwm:
            smod = -1 # left
        else:
            smod =  1 # right


        if self.curr_smod is None:
            self.curr_smod = smod
        else:
            if smod != self.curr_smod:
                self.prev_smod = self.curr_smod
                self.curr_smod = smod
                self.smod_changed = True

        # throttle
        if abs(throttle_pwm - throttle_mid_pwm) < self.throttle_pwm_threshold:
            tmod =  0 # stop
        elif throttle_pwm >= throttle_mid_pwm:
            tmod =  1 # drive forward
        else:
            tmod = -1 # drive backward

        if self.curr_tmod is None:
            self.curr_tmod = tmod
        else:
            if tmod != self.curr_tmod:
                self.prev_tmod = self.curr_tmod
                self.curr_tmod = tmod
                self.tmod_changed = True


        # mode
        if abs(mode_min_pwm - mode_pwm) < self.mode_pwm_threshold:
            mod = -1
        elif abs(mode_mid_pwm - mode_pwm) < self.mode_pwm_threshold:
            mod =  0
        elif abs(mode_max_pwm - mode_pwm) < self.mode_pwm_threshold:
            mod =  1
        else:
            self.is_connected = False
            return
        
        if self.curr_mod is None:
            self.curr_mod = mod
        else:
            if mod != self.curr_mod:
                self.prev_mod = self.curr_mod
                self.curr_mod = mod
                self.mod_changed = True


    def mod_command(self):
        if self.curr_mod ==  1: # top
            self.lit.headlights_on()
        if self.curr_mod ==  0: # middle
            self.lit.headlights_off()
        if self.curr_mod == -1: # bottom
            self.lit.viz_status(c = 'c', cycles = -1)


    def smod_command(self):
        if self.curr_smod == -1: # left
            self.lit.indic_left_on()
        if self.curr_smod ==  0: # straight
            self.lit.indic_off()
        if self.curr_smod ==  1: # right
            self.lit.indic_right_on()


    def tmod_command(self):
        if self.curr_tmod == -1: # backward
            self.lit.release_brake()
        if self.curr_tmod ==  0: # stop
            self.lit.push_brake(duration_s = 1)
        if self.curr_tmod ==  1: # farward
            self.lit.release_brake()

    def callback(self,data):

        self.parse_pwm(data.data)

        if not self.is_connected: return

        if self.mod_changed:
            rospy.loginfo("Mode changed!")
            self.mod_command()
            self.mod_changed = False

        if self.smod_changed:
            rospy.loginfo("Steering mode changed!")
            self.smod_command()
            self.smod_changed = False

        if self.tmod_changed:
            rospy.loginfo("Driving mode changed!")
            self.tmod_command()
            self.tmod_changed = False


if __name__ == '__main__':

    # initialize node
    rospy.init_node('rc_commands', anonymous=True, disable_signals = False)

    rc = rc_commands()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down rc_commands")
