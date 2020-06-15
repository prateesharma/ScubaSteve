####################################
# 2019-2020, Lockheed Martin Space #
# TDC Team #2, Scuba Squad         #
####################################

import rospy
import smach

from scuba_steve.state_machine.states.powerdown_state import PowerdownState
from scuba_steve.state_machine.states.surface_state import SurfaceState


def create_powerdown_sm():
    rospy.loginfo(f"Building state machine 'POWERDOWN'")

    # Add states to the state machine
    sm = smach.StateMachine(outcomes=['outcome_sm'])
    with sm:
        smach.StateMachine.add(
            'SURFACE',
            SurfaceState(),
            transitions={'outcome1':'POWERDOWN'}
        )
        smach.StateMachine.add(
            'POWERDOWN',
            PowerdownState(),
            transitions={'outcome1':'outcome_sm'}
        )
    return sm
