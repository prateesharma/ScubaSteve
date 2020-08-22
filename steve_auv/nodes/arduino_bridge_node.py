####################################
# 2019-2020, Lockheed Martin Space #
# TDC Team #2, Scuba Squad         #
####################################

#!/usr/bin/env python

import rospy
import steve_auv.arduino_bridge as ab

from ab.bridge.arduino_bridge import ArduinoBridge


def main():
    rospy.init_node('arduino_bridge')

    # Set up and run the Arduino bridge
    rospy.loginfo("Executing Arduino bridge")
    bridge = ArduinoBridge(
                 rospy.get_name(),
                 rospy.get_param('~gnc_thrusters_topic')
             )
    rospy.spin()


if __name__ == '__main__':
    main()
