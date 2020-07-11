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
        succeeded: 'END'
    """
    def __init__(self, name='POWERDOWN'):
        smach.State.__init__(self, outcomes=['succeeded'])
        self.name = name

    def execute(self, userdata):
        rospy.loginfo(f"Executing state '{self.name}'")

        # TODO - Save data
        # TODO - Kill processes
        return 'succeeded'
