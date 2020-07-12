####################################
# 2019-2020, Lockheed Martin Space #
# TDC Team #2, Scuba Squad         #
####################################

import rospy
import smach
import steve_auv.mission_manager as mm

from mm.states.powerdown_state import PowerdownState
from mm.states.surface_state import SurfaceState


def build_powerdown_sm():
    rospy.loginfo(f"Building 'POWERDOWN' state machine")

    # Add states to the state machine
    sm = smach.StateMachine(outcomes=['succeeded'])
    with sm:
        smach.StateMachine.add(
            'SURFACE',
            SurfaceState(),
            transitions={'succeeded':'POWERDOWN', failed:'POWERDOWN'}
        )
        smach.StateMachine.add(
            'POWERDOWN',
            PowerdownState(),
            transitions={'succeeded':'succeeded'}
        )
    return sm
