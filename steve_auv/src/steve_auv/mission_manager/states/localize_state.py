####################################
# 2019-2020, Lockheed Martin Space #
# TDC Team #2, Scuba Squad         #
####################################

import actionlib
import rospy
import smach

from steve_auv.msg import LocalizeAction, LocalizeGoal


class LocalizeState(smach.State):
    """State where the vehicle is in the water and attempts to localize itself
    within the environment relative to its entrypoint.

    State: 'LOCALIZE'

    Outcomes:
        succeeded: 'IDLE'
        failed:    'TERMINATE'
    """
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        self.is_localized = False
        self.timeout = 300  # seconds until timeout

    def execute(self, userdata):
        rospy.loginfo(f"Executing state 'LOCALIZE'")

        # Set up a client and connect to the action server
        client = actionlib.SimpleActionClient(
                     userdata.topics.localize,
                     LocalizeAction
                 )
        connect = client.wait_for_server()
        if not connect:
            rospy.logerr(f"Localize server timeout: failure, powering down")
            userdata.is_failed = True
            return 'failed'

        # Wait until the localize signal is received
        goal = LocalizeGoal()
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
