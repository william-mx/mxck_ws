#!/usr/bin/env python

import rospy
from std_msgs.msg import UInt16MultiArray
from collections import OrderedDict

class LightingControl:

    def __init__(self):
	
	    # basic colors rgb
        self.basic_colors = {
        'r': (255,   0,   0), # red
        'g': (  0, 255,   0), # green
        'b': (  0,   0, 255), # blue
        'c': (  0, 255, 255), # cyan
        'm': (255,   0, 255), # magenta
        'y': (255,  70,   0), # yellow
        'o': (255,  40,   0), # orange
        'p': (255,   5,   5), # pink
        'w': (255,  80,  80)} # white

        # define "infinite" number of cycles
        self.c_inf = 65534 # uint16 max

        # subscribe vehicle lighting and status led
        self.lights_pub = rospy.Publisher("/lights",UInt16MultiArray, queue_size=100)
        self.status_pub = rospy.Publisher("/brake_light",UInt16MultiArray, queue_size=100)

        # readable config format
        self.lights_config = OrderedDict()
        self.lights_config['headlights'] = 0 # (0) off, (1) on, (2) high beam only
        self.lights_config['highbeam_ms_on'] = 0
        self.lights_config['highbeam_ms_off'] = 0
        self.lights_config['highbeam_cycles'] = 0
        self.lights_config['indicators'] = 0 # (0) off, (1) left, (2) right, (3) hazard
        self.lights_config['indicators_ms_on'] = 0
        self.lights_config['indicators_ms_off'] = 0
        self.lights_config['indicator_cycles'] = 0
        self.lights_config['rear_light'] = 0 # (0) off, (1) breake light, (2) + additional breake light


        self.status_config = OrderedDict()
        self.status_config['R'] = 0
        self.status_config['G'] = 0
        self.status_config['B'] = 0
        self.status_config['ms_on'] = 0
        self.status_config['ms_off'] = 0
        self.status_config['cycles'] = 0 # (-1) inf loop, (0) contineously on, (N) N loops


        # initialize messages
        self.lights_msg = UInt16MultiArray()
        self.status_msg = UInt16MultiArray()

        self.lights_msg.data = list(self.lights_config.values())
        self.status_msg.data = list(self.status_config.values())


    def update_lights(self):
        self.lights_msg.data = list(self.lights_config.values())
        rospy.loginfo("Publishing %s to /lights.", self.lights_config.values())
        self.lights_pub.publish(self.lights_msg)

    def update_status(self):
        self.status_msg.data = list(self.status_config.values())
        rospy.loginfo("Publishing %s to /brake_lights.", self.status_config.values())
        self.status_pub.publish(self.status_msg)

    def lights_off(self):
        # turn all lights off
        for k in self.lights_config.keys():
            self.lights_config[k] = 0
        for k in self.status_config.keys():
            self.status_config[k] = 0

        self.update_lights()
        self.update_status()

    def headlights_on(self):
        self.lights_config['headlights'] = 1
        self.update_lights()

    def headlights_off(self):
        self.lights_config['headlights'] = 0
        self.update_lights()

    def highbeam_on(self):
        self.lights_config['headlights'] = 2
        self.lights_config['highbeam_ms_on'] = 0
        self.lights_config['highbeam_ms_off'] = 0 
        self.lights_config['highbeam_cycles'] = 0
        self.update_lights()

    def set_indic_pulse(self, ms_on, ms_off, cycles):
        self.lights_config['indicators_ms_on'] = ms_on
        self.lights_config['indicators_ms_off'] = ms_off
        self.lights_config['indicator_cycles'] = cycles or self.c_inf

    def indic_off(self):
        self.lights_config['indicators'] = 0
        self.set_indic_pulse(0, 0, 0)
        self.update_lights()

    def indic_left_on(self, ms_on = 1000, ms_off = 1000, cycles = None):
        self.lights_config['indicators'] = 1
        self.set_indic_pulse(ms_on, ms_off, cycles)
        self.update_lights()

    def indic_right_on(self, ms_on = 1000, ms_off = 1000, cycles = None):
        self.lights_config['indicators'] = 2
        self.set_indic_pulse(ms_on, ms_off, cycles)
        self.update_lights()

    def warning_lights_on(self, ms_on = 1000, ms_off = 1000, cycles = None):
        self.lights_config['indicators'] = 3
        self.set_indic_pulse(ms_on, ms_off, cycles)
        self.update_lights()

    def headlight_flash(self, duration = None):
        if duration is None: return
        
        self.lights_config['headlights'] = 2
        self.lights_config['highbeam_ms_on'] = duration
        self.lights_config['highbeam_ms_off'] = 0
        self.lights_config['highbeam_cycles'] = 1
        self.update_lights()

    def push_brake(self, add_brake_light = True, duration_s = None):

        self.lights_config['rear_light'] = 2 if add_brake_light else 1
        self.update_lights()

        if not duration_s is None:
            rospy.sleep(duration_s)
            self.release_brake()

    def release_brake(self):

        self.lights_config['rear_light'] = 0
        self.update_lights()

    def release_brake(self):

        self.lights_config['rear_light'] = 0
        self.update_lights()
	

    def set_status(self, c = 'r', ms_on = 1000, ms_off = 1000, cycles = None):

        cycles = self.c_inf if cycles == -1 else cycles # inf loop

        self.status_config['R'] = self.basic_colors[c][0]
        self.status_config['G'] = self.basic_colors[c][1]
        self.status_config['B'] = self.basic_colors[c][2]
        self.status_config['ms_on'] = ms_on
        self.status_config['ms_off'] = ms_off
        self.status_config['cycles'] = cycles or 0 # continously on

        self.update_status()
