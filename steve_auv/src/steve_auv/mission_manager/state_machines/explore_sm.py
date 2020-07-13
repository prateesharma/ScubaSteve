####################################
# 2019-2020, Lockheed Martin Space #
# TDC Team #2, Scuba Squad         #
####################################

import rospy
import smach
import steve_auv.mission_manager as mm

from mm.states.powerdown_state import PowerdownState
from mm.states.surface_state import SurfaceState


def build_explore_sm():
    """Builds the state machine for the explore sequence."""
    rospy.loginfo(f"Building 'EXPLORE' state machine")

    # Add states to the state machine
    sm = smach.StateMachine(outcomes=['succeeded'])
    with sm:
        smach.StateMachine.add(
            'DIVE',
            SimpleActionState(
                gnc_topic,
                GncAction,
                goal=GncGoal('dive'),
                result_cb=action_cb,
                exec_timeout=rospy.Duration(60.0),
                server_wait_timeout=rospy.Duration(10.0)
            ),
            transitions={'succeeded':'POWERDOWN'}
        )
        smach.StateMachine.add(
            'EXPLORE',
            SimpleActionState(
                gnc_topic,
                GncAction,
                goal=GncGoal('explore'),
                result_cb=action_cb,
                exec_timeout=None,
                server_wait_timeout=rospy.Duration(10.0)
            ),
            transitions={'succeeded':'POWERDOWN'}
        )
    return sm
