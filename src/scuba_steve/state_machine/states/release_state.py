####################################
# 2019-2020, Lockheed Martin Space #
# TDC Team #2, Scuba Squad         #
####################################

import rospy
import smach

from scuba_steve.state_machine.utils.mission_clock import MissionClock


class ReleaseState(smach.State):
    """State where the system remains ready until it receives the "release"
    signal from the Ground Station. Upon receiving the "release" signal, the
    system starts the mission clock.

    State: 'RELEASE'

    Outcomes:
        outcome1: 'SPLASHDOWN'
    """
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1'])

    def execute(self, userdata):
        rospy.loginfo("Executing state 'RELEASE'")

        # TODO - Remain in this state until the release signal is received
        # Start the mission clock
        mc = MissionClock.get_instance()
        return 'outcome1'
