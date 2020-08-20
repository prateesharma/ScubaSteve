####################################
# 2019-2020, Lockheed Martin Space #
# TDC Team #2, Scuba Squad         #
####################################

#!/usr/bin/env python

import rospy
import steve_auv.vision as vision

from vision.server.vision_server import VisionServer


def main():
    rospy.init_node('vision')

    # Set up and run the vision server
    rospy.loginfo("Executing vision server")
    server = VisionServer(
                 rospy.get_name(),
                 rospy.get_param('~vision_mode_topic'),
                 rospy.get_param('~vision_output_dir'),
                 rospy.get_param('~vision_rate')
             )
    rospy.spin()


if __name__ == '__main__':
    main()
