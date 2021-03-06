####################################
# 2019-2020, Lockheed Martin Space #
# TDC Team #2, Scuba Squad         #
####################################

import rospy
import smach
import smach_ros
import steve_auv.mission_manager as mm

from mm.states.explore_state import ExploreState
from mm.utils import action_cb
from steve_auv.msg import GncAction, GncGoal


def build_explore_demo_sm(topics):
    """Builds the state machine for the explore sequence."""
    rospy.loginfo(f"Building 'EXPLORE' state machine")

    # Add states to the state machine
    sm = smach.StateMachine(
             outcomes=['succeeded', 'failed']
         )
    with sm:
        smach.StateMachine.add(
            'DIVE',
            smach_ros.SimpleActionState(
                topics.gnc_mode_topic,
                GncAction,
                goal=GncGoal('dive'),
                result_cb=action_cb,
                exec_timeout=rospy.Duration(60.0),
                server_wait_timeout=rospy.Duration(10.0)
            ),
            transitions={'succeeded':'EXPLORE', 'failed':'failed'}
        )
        smach.StateMachine.add(
            'EXPLORE',
            ExploreState(
                topics.gnc_mode_topic,
                topics.vision_mode_topic,
                preempt_timeout=rospy.Duration(10.0),
                server_wait_timeout=rospy.Duration(10.0)
            ),
            transitions={'succeeded':'SURFACE', 'preempted':'SURFACE', 'failed':'failed'}
        )
        smach.StateMachine.add(
            'SURFACE',
            smach_ros.SimpleActionState(
                topics.gnc_mode_topic,
                GncAction,
                goal=GncGoal('surface'),
                result_cb=action_cb,
                exec_timeout=rospy.Duration(60.0),
                server_wait_timeout=rospy.Duration(10.0)
            ),
            transitions={'succeeded':'succeeded', 'failed':'failed'}
        )
    return sm
