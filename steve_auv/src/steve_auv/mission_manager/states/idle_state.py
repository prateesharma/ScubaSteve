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
        failed:    'FAIL_POWERDOWN'
    """
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['explore', 'comms', 'powerdown', 'failed']
        )
        self.mc = MissionClock.get_instance()

    def execute(self, userdata):
        rospy.loginfo("Executing state 'IDLE'")

        # Act if there is a command
        if userdata.command == "continue":
            rospy.loginfo("Continue command received: continue")
        elif userdata.command == "kill":
            rospy.loginfo("Kill command received: powering down")
            return 'powerdown'
        if not userdata.command:
            rospy.loginfo("No command received: continue")
        else:
            rospy.logerr("Unrecognized command received: continue anyway")

        # Reference the mission clock against the mission schedule to determine
        # the next state
        state = self.mc.get_mission_state()
        if state == 'EXPLORE':
            return 'explore'
        elif state == 'COMMS':
            return 'comms'
        elif state == 'POWERDOWN':
            return 'powerdown'
        else:
            rospy.logerr("Unrecognized state: failure, powering down")
            userdata.is_failed = True
            return 'failed'
