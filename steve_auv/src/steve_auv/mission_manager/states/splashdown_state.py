####################################
# 2019-2020, Lockheed Martin Space #
# TDC Team #2, Scuba Squad         #
####################################

import rospy
import smach
import std_msgs


class SplashdownState(smach.State):
    """State where the vehicle is released from the Launch Structure into the
    water. The system determines when the vehicle hits the water and attempts to
    stabilize the vehicle at the surface of the water.

    State: 'SPLASHDOWN'

    Outcomes:
        succeeded: 'LOCALIZE'
        failed:    'POWERDOWN'
    """
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        self.is_released = False
        self.timeout = 300  # seconds until timeout

    def execute(self, userdata):
        rospy.loginfo(f"Executing state 'SPLASHDOWN'")

        # Subscribe to the GN&C module for the splashdown topic
        def splashdown_cb(msg):
            self.is_splashdown = msg.data

        rospy.Subscriber(
            userdata.topics.splashdown,
            std_msgs.msg.Bool,
            splashdown_cb
        )

        # Remain in this state until the splashdown signal is received
        for t in range(self.timeout):
            # If received, continue
            if self.is_splashdown:
                rospy.loginfo(f"Received splashdown signal: continuing")
                return 'succeeded'
            else:
                rospy.sleep(1)

        # Return a failure after the timeout
        rospy.logerr(f"Splashdown timeout: failure, powering down")
        return 'failed'
