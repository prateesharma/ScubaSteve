####################################
# 2019-2020, Lockheed Martin Space #
# TDC Team #2, Scuba Squad         #
####################################

import rospy
import smach


class SplashdownState(smach.State):
    """State where the vehicle is released from the Launch Structure into the
    water. The system determines when the vehicle hits the water and attempts to
    stabilize the vehicle at the surface of the water and get an initial
    localization and bearing.

    State: 'SPLASHDOWN'

    Outcomes:
        outcome1: 'IDLE'
    """
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1'])

    def execute(self, userdata):
        rospy.loginfo("Executing state 'SPLASHDOWN'")

        # TODO - Read from the 'GN&C' node to determine when the vehicle impacts
        #        with the water.
        # TODO - Wait for the vehicle to stabilize and come to a rest
        # TODO - Calculate an initial localization and bearing
        return 'outcome1'
