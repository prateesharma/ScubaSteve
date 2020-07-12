####################################
# 2019-2020, Lockheed Martin Space #
# TDC Team #2, Scuba Squad         #
####################################

import rospy
import smach


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

        # Subscribe to the GN&C module for the localize topic
        def localize_cb(msg):
            self.is_localized = msg.data

        rospy.Subscriber(
            userdata.topics.localize,
            std_msgs.msg.Bool,
            localize_cb
        )

        # Remain in this state until the localize signal is received
        for t in range(self.timeout):
            # If received, continue
            if self.is_localized:
                rospy.loginfo(f"Received localize signal: continuing")
                return 'succeeded'
            else:
                rospy.sleep(1)

        # Return a failure after the timeout
        rospy.logerr(f"Localize timeout: failure, powering down")
        userdata.is_failed = True
        return 'failed'
