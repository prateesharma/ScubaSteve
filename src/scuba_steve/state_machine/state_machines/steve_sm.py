####################################
# 2019-2020, Lockheed Martin Space #
# TDC Team #2, Scuba Squad         #
####################################

import rospy
import smach

from scuba_steve.state_machine.state_machines.powerdown_sm import create_powerdown_sm
from scuba_steve.state_machine.states.end_state import EndState
from scuba_steve.state_machine.states.idle_state import IdleState
from scuba_steve.state_machine.states.powerup_state import PowerupState
from scuba_steve.state_machine.states.release_state import ReleaseState
from scuba_steve.state_machine.states.splashdown_state import SplashdownState
from scuba_steve.state_machine.states.start_state import StartState


def create_steve_sm():
    rospy.loginfo(f"Building state machine 'STEVE'")

    # Add states to an empty state machine
    sm = smach.StateMachine(outcomes=['outcome_sm'])
    with sm:
        smach.StateMachine.add(
            'START',
            StartState(),
            transitions={'outcome1':'POWERUP'}
        )
        smach.StateMachine.add(
            'POWERUP',
            PowerupState(),
            transitions={'outcome1':'RELEASE'}
        )
        smach.StateMachine.add(
            'RELEASE',
            ReleaseState(),
            transitions={'outcome1':'SPLASHDOWN'}
        )
        smach.StateMachine.add(
            'SPLASHDOWN',
            SplashdownState(),
            transitions='outcome1':'IDLE'}
        )
        smach.StateMachine.add(
            'IDLE',
            IdleState(),
            transitions='outcome1':'POWERDOWN'}
        )
        smach.StateMachine.add(
            'POWERDOWN',
            create_powerdown_sm(),
            transitions={'outcome_sm':'END'}
        )
        smach.StateMachine.add(
            'END',
            EndState(),
            transitions={'outcome1':'outcome_sm'}
        )
    return sm
