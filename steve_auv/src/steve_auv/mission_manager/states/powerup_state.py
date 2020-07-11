####################################
# 2019-2020, Lockheed Martin Space #
# TDC Team #2, Scuba Squad         #
####################################

import rospy
import smach


class PowerupState(smach.State):
    """State that initializes the system.

    State: 'POWERUP'

    Outcomes:
        succeeded: 'RELEASE'
    """
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])

    def execute(self, userdata):
        rospy.loginfo(f"Executing state 'POWERUP'")

        # TODO - Startup and initialization processes should be executed here
        return 'succeeded'
