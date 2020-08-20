####################################
# 2019-2020, Lockheed Martin Space #
# TDC Team #2, Scuba Squad         #
####################################

import actionlib
import rospy
import smach
import steve_auv.mission_manager as mm

from mm.utils.mission_clock import MissionClock
from steve_auv.msg import GncAction, GncGoal


class ExploreState(smach.State):
    """State where an explore action is given to another subsystem via an action
    client. Based on the schedule and the mission clock, the client is given a
    timeout, after which, the action is preempted. After the action is either
    completed or preempted, the mission clock's state is incremented.

    State: ExploreState

    Outcomes:
        succeeded: The goal was completed successfully
        preempted: The goal was preempted successfully
        failed:    The goal failed to be completed or preempted by the server
    """
    def __init__(self, gnc_action_name, vision_action_name,
                 preempt_timeout=rospy.Duration(10.0),
                 server_wait_timeout=rospy.Duration(60.0)):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'preempted', 'failed']
        )
        self.gnc_action_name = gnc_action_name
        self.vision_action_name = vision_action_name
        self.preempt_timeout = preempt_timeout
        self.gnc_server_wait_timeout = gnc_server_wait_timeout
        self.mc = MissionClock.get_instance()

    def execute(self, userdata):
        rospy.loginfo("Executing ExploreState")

        # Set up GNC client and connect to the action server
        gnc_client = actionlib.SimpleActionClient(
                         self.gnc_action_name,
                         GncAction
                     )
        gnc_connect = gnc_client.wait_for_server(self.server_wait_timeout)
        if not gnc_connect:
            rospy.logerr("GNC server timeout: failure, powering down")
            userdata.is_failed = True
            return 'failed'

        # Set up vision client and connect to the action server
        vision_client = actionlib.SimpleActionClient(
                            self.vision_action_name,
                            VisionAction
                        )
        vision_connect = vision_client.wait_for_server(self.server_wait_timeout)
        if not vision_connect:
            rospy.logerr("Vision server timeout: failure, powering down")
            userdata.is_failed = True
            return 'failed'

        # Start the vision server, which will until the GNC server is preempted
        vision_client.send_goal(VisionGoal('poi'))

        # Wait until the GNC goal is completed, otherwise, preempt the goal
        execute_timeout = rospy.Duration(self.mc.get_time_until_next_state())
        gnc_status = gnc_client.send_goal_and_wait(
            GncGoal('explore'),
            execute_timeout=execute_timeout,
            preempt_timeout=self.preempt_timeout
        )

        # Stop the action on the vision server
        vision_client.cancel_goal()
        vision_client.wait_for_result(self.preempt_timeout)

        # Record the number of new images in the userdata
        vision_result = vision_client.get_result()
        userdata.image_count = vision_result.end_idx
        userdata.new_image_index = vision_result.start_idx
        userdata.new_image_count = (vision_result.end_idx
                                    - vision_result.start_idx)

        # Transition to the next state based on the state of the GNC goal
        self._result_cb(userdata, gnc_client.get_result(), gnc_status)
        self.mc.next_state()
        if status == actionlib.GoalStatus.SUCCEEDED:
            return 'succeeded'
        elif status == actionlib.GoalStatus.PREEMPTED:
            return 'preempted'
        else:
            return 'failed' 

    def _result_cb(userdata, status, result):
        """Handles logging and flags for the explore state."""
        if status == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("Explore goal completed successfully: continue")
        elif status == actionlib.GoalStatus.PREEMPTED:
            rospy.loginfo("Explore goal preempted successfully: continue")
        else:
            rospy.logerr("Something went wrong: failure, powering down")
            userdata.is_failed = True
