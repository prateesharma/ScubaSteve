####################################
# 2019-2020, Lockheed Martin Space #
# TDC Team #2, Scuba Squad         #
####################################

import actionlib
import rospy
import smach
import steve_auv.mission_manager as mm

from mm.utils.mission_clock import MissionClock
from steve_auv.msg import ReleaseAction, ReleaseGoal


class ReleaseState(smach.State):
    """State where the system remains ready until it receives the "release"
    signal from the Ground Station. Upon receiving the "release" signal, the
    system starts the mission clock.

    State: 'RELEASE'

    Outcomes:
        succeeded:   'SPLASHDOWN'
        failed:      'TERMINATE'
    """
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        self.is_released = False
        self.timeout = rospy.Duration(1800)  # seconds until timeout

    def execute(self, userdata):
        rospy.loginfo(f"Executing state 'RELEASE'")

        # Set up a client and connect to the action server
        client = actionlib.SimpleActionClient(
                     userdata.topics.release,
                     ReleaseAction
                 )
        connect = client.wait_for_server()
        if not connect:
            rospy.logerr(f"Release server timeout: failure, powering down")
            userdata.is_failed = True
            return 'failed'

        # Wait until the release signal is received
        goal = ReleaseGoal()
        client.send_goal(goal)
        result = client.wait_for_result(self.timeout)

        # If received, start the mission clock and continue
        if result:
            rospy.loginfo(f"Received release signal: starting clock")
            mc = MissionClock.get_instance()
            mc.reset()
            return 'succeeded'
        else:
            rospy.logerr(f"Release goal timeout: failure, powering down")
            userdata.is_failed = True
            return 'failed'
