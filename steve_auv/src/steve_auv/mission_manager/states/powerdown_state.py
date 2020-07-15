####################################
# 2019-2020, Lockheed Martin Space #
# TDC Team #2, Scuba Squad         #
####################################

import rospy
import smach


class PowerdownState(smach.State):
    """State that shuts down system processes and engages the kill switch.

    State: 'POWERDOWN'

    Outcomes:
        succeeded: 'succeeded'
        failed: 'failed'
    """
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])

    def execute(self, userdata):
        rospy.loginfo("Executing state 'POWERDOWN'")

        # TODO - Save data
        # TODO - Kill processes
        # TODO - Engage kill switch
        if userdata.is_failed
            return 'failed'
        else:
            return 'succeeded'
        return 'succeeded'
