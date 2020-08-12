####################################
# 2019-2020, Lockheed Martin Space #
# TDC Team #2, Scuba Squad         #
####################################

import actionlib
import rospy
import steve_auv.gnc as gnc

from steve_auv.msg import GncAction, GncResult


class GncDemoServer(object):
    """GNC action server that listens for goals from the mission manager and
    acts upon them when received. The GNC server can work on one of the
    following processes at a time:

        'dive':    Vehicle dives underwater by thrusting downward.
        'explore': Vehicle explores the environment by thrusting forward.
        'surface': Vehicle surfaces by thrusting upward.

    Processes can be preempted or cancelled by the mission manager.
    """
    def __init__(self, name, topic, host, port):
        self._name = name
        self._topic = topic
        self._server = actionlib.SimpleActionServer(
                           self._topic,
                           GncAction,
                           execute_cb=self.execute_cb
                       ).start()

    def execute_cb(self, goal):
        is_success = False
        result = GncResult()
        if goal.action == "release":
            rate = rospy.Rate(1)
            while True:
                if self._server.is_preempt_requested():
                    break
                cmd = self._socket.listen()
                if cmd == "release":
                    is_success = True
                    break
                rate.sleep() 
        elif goal.action == "comms":
             rate = rospy.Rate(1)
             while True:
                if self._server.is_preempt_requested():
                    result.command = "continue"
                    break
                cmd = self._socket.listen()
                if cmd == "downlink":
                    # TODO: Copy files
                    pass
                elif cmd == "continue":
                    result.command = "continue"
                    is_success = True
                    break
                elif cmd == "kill":
                    result.command = "kill"
                    is_success = True
                    break
                rate.sleep() 
        else:
            rospy.logerr("Invalid goal received. Cancelling goal.")

        if is_success:
            self._server.set_succeeded(result)
        else:
            self._server.set_preempted(result)
