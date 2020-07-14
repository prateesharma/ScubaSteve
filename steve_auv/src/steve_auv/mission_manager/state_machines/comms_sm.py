####################################
# 2019-2020, Lockheed Martin Space #
# TDC Team #2, Scuba Squad         #
####################################

import rospy
import smach
import steve_auv.mission_manager as mm

from mm.states.powerdown_state import PowerdownState
from mm.states.surface_state import SurfaceState


def build_comms_sm():
    """Builds the state machine for the comms sequence."""
    rospy.loginfo(f"Building 'COMMS' state machine")

    # Add states to the state machine
    sm = smach.StateMachine(outcomes=['succeeded', 'failed'])
    with sm:
        smach.StateMachine.add(
            'SURFACE',
            SimpleActionState(
                gnc_topic,
                GncAction,
                goal=GncGoal('surface'),
                result_cb=action_cb,
                exec_timeout=rospy.Duration(60.0),
                server_wait_timeout=rospy.Duration(10.0)
            ),
            transitions={'succeeded':'COMMS', 'failed':'failed'}
        )
        smach.StateMachine.add(
            'COMMS',
            SimpleActionState(
                comms_topic,
                CommsAction,
                goal=CommsGoal('transfer'),
                result_cb=action_cb,
                exec_timeout=None,
                server_wait_timeout=rospy.Duration(10.0)
            ),
            transitions={'succeeded':'succeeded', 'failed':'failed'}
        )
    return sm
