####################################
# 2019-2020, Lockheed Martin Space #
# TDC Team #2, Scuba Squad         #
####################################

import actionlib
import rospy
import smach

from steve_auv.msg import SplashdownAction, SplashdownGoal


class SplashdownState(smach.State):
    """State where the vehicle is released from the Launch Structure into the
    water. The system determines when the vehicle hits the water and attempts to
    stabilize the vehicle at the surface of the water.

    State: 'SPLASHDOWN'

    Outcomes:
        succeeded: 'LOCALIZE'
        failed:    'TERMINATE'
    """
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        self.is_released = False
        self.timeout = 300  # seconds until timeout

    def execute(self, userdata):
        rospy.loginfo(f"Executing state 'SPLASHDOWN'")

        # Set up a client and connect to the action server
        client = actionlib.SimpleActionClient(
                     userdata.topics.splashdown,
                     SplashdownAction
                 )
        connect = client.wait_for_server()
        if not connect:
            rospy.logerr(f"Splashdown server timeout: failure, powering down")
            userdata.is_failed = True
            return 'failed'

        # Wait until the splashdown signal is received
        goal = SplashdownGoal()
        client.send_goal(goal)
        result = client.wait_for_result(self.timeout)

        # If received, continue
        if result:
            rospy.loginfo(f"Received splashdown signal: continuing")
            return 'succeeded'
        else:
            rospy.logerr(f"Splashdown goal timeout: failure, powering down")
            userdata.is_failed = True
            return 'failed'
