####################################
# 2019-2020, Lockheed Martin Space #
# TDC Team #2, Scuba Squad         #
####################################

import rospy
import smach


class IdleState(smach.State):
    """Neutral state where the vehicle checks the mission clock and references
    it against the mission timeline to determine its next state transition.

    State: 'IDLE'

    Outcomes:
        outcome1: 'POWERDOWN'
    """
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['outcome1']
        )

    def execute(self, userdata):
        rospy.loginfo("Executing state 'IDLE'")

        # TODO - Check the mission clock
        # TODO - Reference the mission clock time against the mission timeline
        #        to determine the transition state
        return 'outcome1'
