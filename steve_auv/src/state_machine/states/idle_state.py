####################################
# 2019-2020, Lockheed Martin Space #
# TDC Team #2, Scuba Squad         #
####################################

import rospy
import smach

from scuba_steve.state_machine.utils.mission_clock import MissionClock


class IdleState(smach.State):
    """Neutral state where the vehicle checks the mission clock and references
    it against the mission timeline to determine its next state transition.

    State: 'IDLE'

    Outcomes:
        outcome1: 'POWERDOWN'
    """
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1'])

    def execute(self, userdata):
        rospy.loginfo("Executing state 'IDLE'")

        # Reference the mission clock against the mission schedule to determine
        # the next state
        mc = MissionClock.get_instance()
        state = mc.get_mission_state()
        # TODO - Hard code 'outcome1' until other states are added
        if state == 'EXPLORE':
            return 'outcome1'
        elif state == 'COMMS':
            return 'outcome1'
        else:
            return 'outcome1'
