####################################
# 2019-2020, Lockheed Martin Space #
# TDC Team #2, Scuba Squad         #
####################################

import actionlib
import rospy

from steve_auv.msg import CommsAction


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
        result = CommsResult()
        if goal.action == "release":
            while True:
                if self._server.is_preempt_requested():
                    self._server.set_preempted()
                    is_success = False
                    break
                # TODO: TCP connection
            
        if goal.action == "comms":
             while True:
                if self._server.is_preempt_requested():
                    self._server.set_preempted()
                    is_success = False
                    break
                # TODO: TCP connection
                if msg:
                    result.command = msg
                    is_success = True
                    break
        else:
            rospy.logerr()
        
        if is_success:
            self._server.set_succeeded(result)



        rate = rospy.Rate(1)

        for i in range(0, goal.number_of_minutes):
            if self.a_server.is_preempt_requested():
                self.a_server.set_preempted()
                success = False
                break

            last_dish_washed = 'bowl-' + str(i)
            feedback.last_dish_washed = last_dish_washed
            result.dishes_washed.append(last_dish_washed)
            self.a_server.publish_feedback(feedback)
            rate.sleep()

        if success:
            self.a_server.set_succeeded(result)
