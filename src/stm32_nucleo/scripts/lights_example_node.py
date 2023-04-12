#!/usr/bin/env python

import sys
import rospkg
import rospy
import os

# add the file path of the package so that interpeter can find the module
r = rospkg.RosPack()
sys.path.append(os.path.join(r.get_path('stm32_nucleo'), 'scripts'))

from lights_control import LightingControl

if __name__ == '__main__':

    # initialize node
    rospy.init_node('lights_control', anonymous=True)

    lit = LightingControl()

    # turn all lights off
    lit.lights_off()
    rospy.sleep(2) # sleep for 2 seconds

    # turn highbeam on
    lit.highbeam_on()
    rospy.sleep(2)

    # turn warning lights on
    lit.warning_lights_on(ms_on = 500, ms_off = 500, cycles = 6)
    rospy.sleep(6)
    lit.lights_off()

    # push break
    lit.push_brake(duration_s = 3)
    rospy.sleep(2)

    # turn all lights off
    lit.lights_off()
    rospy.sleep(2) 

    # switch different status led colors
    lit.set_status(c = 'r', ms_on = 500, ms_off = 500, cycles = 4)
    rospy.sleep(4)
    lit.set_status(c = 'g', ms_on = 500, ms_off = 500, cycles = 4)
    rospy.sleep(4)
    lit.set_status(c = 'b', ms_on = 500, ms_off = 500, cycles = 4)
    rospy.sleep(4)
    lit.set_status(c = 'm', ms_on = 500, ms_off = 500, cycles = 4)
    rospy.sleep(4)

    # turn all lights off
    lit.lights_off()
    rospy.sleep(2) # sleep for  seconds

    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting lighting control")