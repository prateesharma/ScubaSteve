####################################
# 2019-2020, Lockheed Martin Space #
# TDC Team #2, Scuba Squad         #
####################################

import actionlib
import rospy
import steve_auv.gnc as gnc
import time

from steve_auv.msg import GncAction, GncResult, GncThrustersMsg


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
                                        GncThrustersMsg
                                    )
        msg = GncThrustersMsg(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        self._thrusters_publisher(msg)

    def execute_cb(self, goal):
        is_success = False
        result = GncResult()
        if goal.action == "explore":
            duration = 60.0
            msg = GncThrustersMsg(0.0, 0.0, 0.0, 0.0, 1.0, 1.0)
        elif goal.action == "dive":
            duration = 15.0
            msg = GncThrustersMsg(1.0, 1.0, 1.0, 1.0, 0.0, 0.0)
        elif goal.action == "surface":
            duration = 15.0
            msg = GncThrustersMsg(-1.0, -1.0, -1.0, -1.0, 0.0, 0.0)
        else:
            rospy.logerr("Invalid goal received. Cancelling goal.")
            self._server.set_preempted(result)

        self._thrusters_publisher(msg)
        rate = rospy.Rate(1)
        start_time = time.time()
        while time.time() - start_time > duration:
            if self._server.is_preempt_requested():
                break
            rate.sleep()
        if time.time() - start_time > duration:
            is_success = True

        if is_success:
            self._server.set_succeeded(result)
        else:
            self._server.set_preempted(result)
