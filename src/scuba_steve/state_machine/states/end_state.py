####################################
# 2019-2020, Lockheed Martin Space #
# TDC Team #2, Scuba Squad         #
####################################

import rospy
import smach


class EndState(smach.State):
    """End state of the state machine, where the kill switch is engaged.

    State: 'END'

    Outcomes:
        outcome1: Final state machine transition
    """
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1'])

    def execute(self, userdata):
        rospy.loginfo("Executing state 'END'")

        # TODO - Engage kill switch
        return 'outcome1'
