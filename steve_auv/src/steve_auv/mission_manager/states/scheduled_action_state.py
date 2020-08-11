####################################
# 2019-2020, Lockheed Martin Space #
# TDC Team #2, Scuba Squad         #
####################################

import actionlib
import rospy
import smach
import steve_auv.mission_manager as mm

from mm.utils.mission_clock import MissionClock


class ScheduledActionState(smach.State):
    """State where an action is given to another subsystem via an action
    client. Based on the schedule and the mission clock, the client is given a
    timeout, after which, the action is preempted. After the action is either
    completed or preempted, the mission clock's state is incremented.

    State: ScheduledActionState

    Outcomes:
        succeeded: The goal was completed successfully
        preempted: The goal was preempted successfully
        failed:    The goal failed to be completed or preempted by the server
    """
    def __init__(self, action_name, action_spec, goal=None,
                 result_cb=_result_cb, preempt_timeout=rospy.Duration(5.0),
                 server_wait_timeout=rospy.Duration(60.0)):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'preempted', 'failed']
        )
        self.action_name = action_name
        self.action_spec = action_spec
        self.goal = goal
        self.result_cb = result_cb
        self.preempt_timeout = preempt_timeout
        self.server_wait_timeout = server_wait_timeout
        self.mc = MissionClock.get_instance()

    def execute(self, userdata):
        rospy.loginfo("Executing ScheduledActionState")

        # Set up a client and connect to the action server
        client = actionlib.SimpleActionClient(
                     self.action_name,
                     self.action_spec
                 )
        connect = client.wait_for_server(self.server_wait_timeout)
        if not connect:
            rospy.logerr("Server timeout: failure, powering down")
            userdata.is_failed = True
            return 'failed'

        # Wait until the goal is completed, otherwise, preempt the goal
        execute_timeout = rospy.Duration(self.mc.get_time_until_next_state())
        status = client.send_goal_and_wait(
            self.goal,
            execute_timeout=execute_timeout,
            preempt_timeout=self.preempt_timeout
        )

        # Transition to the next state based on the state of the goal
        self._result_cb(userdata, client.get_result(), status)
        self.mc.next_state()
        if status == actionlib.GoalStatus.SUCCEEDED:
            return 'succeeded'
        elif status == actionlib.GoalStatus.PREEMPTED:
            return 'preempted'
        else:
            return 'failed' 

    def _result_cb(self, userdata, status, result):
        if status == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("Goal completed successfully: continue")
        elif status == actionlib.GoalStatus.PREEMPTED:
            rospy.loginfo("Goal preempted successfully: continue")
        else:
            rospy.logerr("Something went wrong: failure, powering down")
            userdata.is_failed = True
