####################################
# 2019-2020, Lockheed Martin Space #
# TDC Team #2, Scuba Squad         #
####################################

#!/usr/bin/env python

import rospy
import smach
import scuba_steve.mission_manager as mm

from mm.state_machines.mission_manager_sm import build_mission_manager_sm


def main():
    rospy.init_node('mission_manager')

    # Create the state machine
    sm = build_mission_manager_sm()

    # Execute the state machine plan
    rospy.loginfo(f"Executing mission manager")
    outcome = sm.execute()
    rospy.spin()

if __name__ == '__main__':
    main()
