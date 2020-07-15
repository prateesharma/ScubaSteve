####################################
# 2019-2020, Lockheed Martin Space #
# TDC Team #2, Scuba Squad         #
####################################

import rospy
import smach
import steve_auv.mission_manager as mm

from mm.utils.mission_clock import MissionClock


class ExploreState(smach.State):
    """State where the vehicle checks the mission clock and references
    it against the mission timeline to determine its next state transition.

    State: 'EXPLORE'

    Outcomes:
        succeeded: 'SURFACE'
        preempted: 'SURFACE'
        failed:    'SURFACE'
    """
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'preempted', 'failed']
        )

    def execute(self, userdata):
        rospy.loginfo(f"Executing state 'EXPLORE'")

        # Set up a client and connect to the action server
        client = actionlib.SimpleActionClient(
                     userdata.topics.gnc,
                     GncAction
                 )
        connect = client.wait_for_server()
        if not connect:
            rospy.logerr(f"Gnc server timeout: failure, powering down")
            userdata.is_failed = True
            return 'failed'

        # Wait until the explore signal is received
        goal = GncGoal('explore')
        client.send_goal(goal)
        result = client.wait_for_result(self.timeout)

        # If received, continue
        if result:
            rospy.loginfo(f"Received localize signal: continuing")
            return 'succeeded'
        else:
            rospy.logerr(f"Localize goal timeout: failure, powering down")
            userdata.is_failed = True
            return 'failed'

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
