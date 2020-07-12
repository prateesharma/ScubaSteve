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
        suceeded: The state machine completed successfully
        failed:   The state machine failed
    """
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])

    def execute(self, userdata):
        rospy.loginfo("Executing state 'END'")

        # TODO - Engage kill switch
        if userdata.is_failed
            return 'failed'
        else:
            return 'succeeded'
