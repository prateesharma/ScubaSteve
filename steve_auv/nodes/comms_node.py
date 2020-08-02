####################################
# 2019-2020, Lockheed Martin Space #
# TDC Team #2, Scuba Squad         #
####################################

#!/usr/bin/env python

import rospy
import steve_auv.comms as comms

from comms.server.comms_server import CommsServer


def main():
    rospy.init_node('comms')

    # Set up and run the comms server
    rospy.loginfo("Executing comms server")
    server = CommsServer(rospy.get_name(), rospy.get_param('~comms_topic'))
    rospy.spin()


if __name__ == '__main__':
    main()
