####################################
# 2019-2020, Lockheed Martin Space #
# TDC Team #2, Scuba Squad         #
####################################

#!/usr/bin/env python

import rospy
import smach

from scuba_steve.state_machine.state_machines.steve_sm import create_steve_sm


def main():
    rospy.init_node('steve_sm')

    # Create the state machine
    sm = create_steve_sm()

    # Execute the state machine plan
    rospy.loginfo("Executing state machine 'STEVE'")
    outcome = sm.execute()
    rospy.spin()

if __name__ == '__main__':
    main()
