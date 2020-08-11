####################################
# 2019-2020, Lockheed Martin Space #
# TDC Team #2, Scuba Squad         #
####################################

import actionlib
import rospy

from steve_auv.msg import CommsAction, CommsResult


class CommsServer(object):
    """Communications actions server that listens for goals from the mission
    manager and acts upon them when received. The comms server can
    work on one of the following processes at a time:

        'release': Waits for the release signal to be sent by the ground
                   station before ending the process.
        'comms':   Allows the ground station to recursively copy image files.
                   Waits for a command to be received from the ground station
                   before ending the process.

    Processes can be preempted or cancelled by the mission manager.
    """
    def __init__(self, name, topic):
        self._name = name
        self._topic = topic
        self._server = actionlib.SimpleActionServer(
                           self._topic,
                           CommsAction,
                           execute_cb=self.execute_cb
                       )
        self._server.start()

    def execute_cb(self, goal):
        is_success = True
        cmd = None
        result = CommsResult()
        if goal.action == "release":
            rate = rospy.Rate(1)
            while True:
                # TODO: TCP connection
                if cmd == "release":
                    self._server.set_succeeded()
                    break
                if self._server.is_preempt_requested():
                    self._server.set_preempted()
                    is_success = False
                    break
                rate.sleep() 
        if goal.action == "comms":
             rate = rospy.Rate(1)
             while True:
                # TODO: TCP connection
                if cmd == "downlink":
                    # TODO: Copy files
                    pass
                if cmd == "continue":
                    result.command = "continue"
                    self._server.set_succeeded()
                    break
                if cmd == "kill":
                    result.command = "kill"
                    self._server.set_succeeded()
                    break
                if self._server.is_preempt_requested():
                    self._server.set_preempted()
                    is_success = False
                    break
                rate.sleep() 
        else:
            rospy.logerr()
        
        if is_success:
            self._server.set_succeeded(result)
        else:
            self._server.set_aborted(result)
