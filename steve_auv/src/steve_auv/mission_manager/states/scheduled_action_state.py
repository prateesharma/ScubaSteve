####################################
# 2019-2020, Lockheed Martin Space #
# TDC Team #2, Scuba Squad         #
####################################

import rospy
import smach
import steve_auv.mission_manager as mm

from mm.utils.mission_clock import MissionClock


class ScheduledActionState(smach.State):
    """State where the vehicle checks the mission clock and references
    it against the mission timeline to determine when to preempt/cancel a
    goal.

    State: ScheduledActionState

    Outcomes:
        succeeded
        preempted
        failed
    """
    def __init__(self, action_name, action_spec, goal=None, result_cb=None,
                 preempt_timeout=rospy.Duration(60.0),
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

        # Wait until the goal is completed or preempted
        client.send_goal(self.goal)
        time_until_preempt = rospy.Duration(self.mc.get_time_until_next_state())
        result = client.wait_for_result(time_until_preempt)
        # TODO

        # If received, continue
        if result:
            rospy.loginfo(f"Received localize signal: continuing")
            return 'succeeded'
        else:
            rospy.logerr(f"Localize goal timeout: failure, powering down")
            userdata.is_failed = True
            return 'failed'

        # Reference the mission clock against the mission schedule to determine
        # the next state
        mc = MissionClock.get_instance()
        state = mc.get_mission_state()
        if state == 'EXPLORE':
            return 'explore'
        elif state == 'COMMS':
            return 'comms'
        elif state == 'POWERDOWN':
            return 'powerdown'
        else:
            rospy.logerr(f"Unrecognized state: failure, powering down")
            userdata.is_failed = True
            return 'failed'
