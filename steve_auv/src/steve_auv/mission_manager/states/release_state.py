####################################
# 2019-2020, Lockheed Martin Space #
# TDC Team #2, Scuba Squad         #
####################################

import rospy
import smach
import std_msgs
import steve_auv.mission_manager as mm

from mm.utils.mission_clock import MissionClock


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
        self.timeout = 1800  # seconds until timeout

    def execute(self, userdata):
        rospy.loginfo(f"Executing state 'RELEASE'")

        # Subscribe to the comms module for the release topic
        def release_cb(msg):
            self.is_released = msg.data

        rospy.Subscriber(
            userdata.topics.release,
            std_msgs.msg.Bool,
            release_cb
        )

        # Remain in this state until the release signal is received
        for t in range(self.timeout):
            # If received, start the mission clock
            if self.is_released:
                rospy.loginfo(f"Received release signal: starting clock")
                mc = MissionClock.get_instance()
                mc.reset()
                return 'succeeded'
            else:
                rospy.sleep(1)
        
        # Return a failure after the timeout
        rospy.logerr(f"Release timeout: failure, powering down")
        return 'failed'
