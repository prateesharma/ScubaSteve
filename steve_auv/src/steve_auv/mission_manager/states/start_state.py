####################################
# 2019-2020, Lockheed Martin Space #
# TDC Team #2, Scuba Squad         #
####################################

import rospy
import smach


class StartState(smach.State):
    """Initial state of the state machine.

    State: 'START'

    Outcomes:
        suceeded: 'POWERUP'
    """
    def __init__(self):
        smach.State.__init__(self, outcomes=['suceeded'])

    def execute(self, userdata):
        rospy.loginfo(f"Executing state 'START'")
        return 'suceeded'
