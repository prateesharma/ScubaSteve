####################################
# 2019-2020, Lockheed Martin Space #
# TDC Team #2, Scuba Squad         #
####################################

import rospy
import smach


class SurfaceState(smach.State):
    """State where the vehicle surfaces if it is underwater.

    State: 'SURFACE'

    Outcomes:
        succeeded: 'IDLE'
        failed:    'POWERDOWN'
    """
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])

    def execute(self, userdata):
        rospy.loginfo("Executing state 'SURFACE'")

        # TODO - Read from the 'GN&C' node to determine when the vehicle is
        #        surfaced
        return 'succeeded'
