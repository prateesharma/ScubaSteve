####################################
# 2019-2020, Lockheed Martin Space #
# TDC Team #2, Scuba Squad         #
####################################

import actionlib
import rospy
import steve_auv.gnc as gnc

from steve_auv.msg import GncAction, GncResult, GncThrusters


class GncDemoServer(object):
    """GNC action server that listens for goals from the mission manager and
    acts upon them when received. The GNC server can work on one of the
    following processes at a time:

        'explore': Vehicle explores the environment by thrusting forward.
        'dive':    Vehicle dives underwater by thrusting downward.
        'surface': Vehicle surfaces by thrusting upward.

    Processes can be preempted or cancelled by the mission manager.
    """
    def __init__(self, name, gnc_mode_topic, gnc_thrusters_topic):
        self._name = name
        self._server = actionlib.SimpleActionServer(
                           self._gnc_topic,
                           GncAction,
                           execute_cb=self.execute_cb
                       ).start()
        self._thrusters_publisher = rospy.Publisher(
                                        gnc_thrusters_topic,
                                        GncThrusters
                                    )

    def execute_cb(self, goal):
        is_success = False
        result = GncResult()
        if goal.action == "explore":
            rate = rospy.Rate(1)
            while True:
                if self._server.is_preempt_requested():
                    break
                # TODO
                rate.sleep() 
        elif goal.action == "dive":
            rate = rospy.Rate(1)
            while True:
                if self._server.is_preempt_requested():
                    break
                # TODO
                rate.sleep()
        elif goal.action == "surface":
            rate = rospy.Rate(1)
            while True:
                if self._server.is_preempt_requested():
                    break
                # TODO
                rate.sleep()
        else:
            rospy.logerr("Invalid goal received. Cancelling goal.")

        if is_success:
            self._server.set_succeeded(result)
        else:
            self._server.set_preempted(result)
