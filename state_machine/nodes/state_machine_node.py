####################################
# 2019-2020, Lockheed Martin Space #
# TDC Team #2, Scuba Squad         #
####################################

#!/usr/bin/env python

import rospy
import smach

from steve_state_machine import steve_state_machine


def main():
    rospy.init_node('steve_sm')

    # Create the state machine
    sm = steve_state_machine('STEVE')

    # Execute the state machine plan
    rospy.loginfo("Executing state machine 'STEVE'")
    outcome = sm.execute()


if __name__ == '__main__':
    main()
