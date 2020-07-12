####################################
# 2019-2020, Lockheed Martin Space #
# TDC Team #2, Scuba Squad         #
####################################

#!/usr/bin/env python

import rospy
import smach
import steve_auv.mission_manager as mm

from mm.state_machines.mission_manager_sm import build_mission_manager_sm


def main():
    rospy.init_node('mission_manager')

    # Access topics from the parameter server
    comms_topic = rospy.get_param('~comms_topic')
    gnc_topic = rospy.get_param('~gnc_topic')

    # Create the state machine
    sm = build_mission_manager_sm(comms_topic, gnc_topic)

    # Execute the state machine plan
    rospy.loginfo(f"Executing mission manager")
    outcome = sm.execute()
    rospy.spin()


if __name__ == '__main__':
    main()
