####################################
# 2019-2020, Lockheed Martin Space #
# TDC Team #2, Scuba Squad         #
####################################

import rospy
import smach


class PowerdownState(smach.State):
    """State that shuts down system processes.

    State: 'POWERDOWN'

    Outcomes:
        outcome1: 'END'
    """
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1'])

    def execute(self, userdata):
        rospy.loginfo("Executing state 'POWERDOWN'")

        # TODO - Save data
        # TODO - Kill processes
        return 'outcome1'
