####################################
# 2019-2020, Lockheed Martin Space #
# TDC Team #2, Scuba Squad         #
####################################

#!/usr/bin/env python

import rospy
import steve_auv.gnc as gnc

from gnc.server.gnc_demo_server import GncDemoServer


def main():
    rospy.init_node('gnc_demo')

    # Set up and run the GNC server
    rospy.loginfo("Executing GNC server")
    server = GncDemoServer(
                 rospy.get_name(),
                 rospy.get_param('~gnc_topic'),
                 rospy.get_param('~arduino_topic')
    )
    rospy.spin()


if __name__ == '__main__':
    main()
