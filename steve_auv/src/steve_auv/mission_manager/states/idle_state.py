####################################
# 2019-2020, Lockheed Martin Space #
# TDC Team #2, Scuba Squad         #
####################################

import rospy
import smach
import steve_auv.mission_manager as mm

from mm.utils.mission_clock import MissionClock


class IdleState(smach.State):
    """Neutral state where the vehicle checks the mission clock and references
    it against the mission timeline to determine its next state transition.

    State: 'IDLE'

    Outcomes:
        explore:   'EXPLORE'
        comms:     'COMMS'
        powerdown: 'POWERDOWN'
        failed:    'POWERDOWN'
    """
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['explore', 'comms', 'powerdown', 'failed']
        )

    def execute(self, userdata):
        rospy.loginfo(f"Executing state 'IDLE'")

        # Reference the mission clock against the mission schedule to determine
        # the next state
        mc = MissionClock.get_instance()
        state = mc.get_mission_state()
        if state == 'EXPLORE':
            return 'explore'
        elif state == 'COMMS':
            return 'comms'
        elif state == 'POWERDOWN':
            return 'powerdown'
        else:
            rospy.logerr(f"Unrecognized state: failure, powering down")
            userdata.is_failed = True
            return 'failed'
