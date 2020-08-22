####################################
# 2019-2020, Lockheed Martin Space #
# TDC Team #2, Scuba Squad         #
####################################

import actionlib
import rospy
import steve_auv.comms as comms

from comms.utils.tcp_socket import TcpSocket
from steve_auv.msg import CommsAction, CommsResult


class CommsServer(object):
    """Communications action server that listens for goals from the mission
    manager and acts upon them when received. The comms server can work on one
    of the following processes at a time:

        'release': Waits for the release signal to be sent by the ground
                   station before ending the process.
        'comms':   Allows the ground station to recursively copy image files.
                   Waits for a command to be received from the ground station
                   before ending the process.

    Processes can be preempted or cancelled by the mission manager.
    """
    def __init__(self, name, topic, host, port):
        self._name = name
        self._server = actionlib.SimpleActionServer(
                           topic,
                           CommsAction,
                           execute_cb=self.execute_cb
                       ).start()
        self._socket = TcpSocket(host, port)

    def execute_cb(self, goal):
        is_success = False
        result = CommsResult()
        if goal.action == "release":
            rate = rospy.Rate(1)
            while True:
                if self._server.is_preempt_requested():
                    break
                if self._socket.has_connection():
                    cmd = self._socket.listen_connection()
                    if cmd == "release":
                        is_success = True
                        break
                else:
                    self._socket.accept_connection()
                rate.sleep()
            self._socket.close_connection()
        elif goal.action == "comms":
             rate = rospy.Rate(1)
             while True:
                if self._server.is_preempt_requested():
                    result.command = "continue"
                    break
                if self._socket.has_connection():
                    cmd = self._socket.listen_connection()
                    if cmd == "downlink":
                        # Do nothing
                        pass
                    elif cmd == "continue":
                        result.command = "continue"
                        is_success = True
                        break
                    elif cmd == "kill":
                        result.command = "kill"
                        is_success = True
                        break
                else:
                    self._socket.accept_connection()
                rate.sleep()
            self._socket.close_connection()
        else:
            rospy.logerr("Invalid goal received. Cancelling goal.")

        if is_success:
            self._server.set_succeeded(result)
        else:
            self._server.set_preempted(result)
