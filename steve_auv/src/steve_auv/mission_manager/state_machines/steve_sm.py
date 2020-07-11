####################################
# 2019-2020, Lockheed Martin Space #
# TDC Team #2, Scuba Squad         #
####################################

import rospy
import smach

from steve_auv.mission_manager.state_machines.powerdown_sm import create_powerdown_sm
from steve_auv.mission_manager.states.end_state import EndState
from steve_auv.mission_manager.states.idle_state import IdleState
from steve_auv.mission_manager.states.localize_state import LocalizeState
from steve_auv.mission_manager.states.powerdown_state import PowerdownState
from steve_auv.mission_manager.states.powerup_state import PowerupState
from steve_auv.mission_manager.states.release_state import ReleaseState
from steve_auv.mission_manager.states.splashdown_state import SplashdownState
from steve_auv.mission_manager.states.start_state import StartState


def build_steve_sm():
    rospy.loginfo(f"Building state machine 'STEVE'")

    # Add states to an empty state machine
    sm = smach.StateMachine(outcomes=['suceeded', 'failed'])
    with sm:
        smach.StateMachine.add(
            'START',
            StartState(),
            transitions={'succeeded':'POWERUP'}
        )
        smach.StateMachine.add(
            'POWERUP',
            PowerupState(),
            transitions={'succeeded':'RELEASE'}
        )
        smach.StateMachine.add(
            'RELEASE',
            ReleaseState(),
            transitions={'suceeded':'SPLASHDOWN', 'failed':'TERMINATE'}
        )
        smach.StateMachine.add(
            'SPLASHDOWN',
            SplashdownState(),
            transitions={'succeeded':'LOCALIZE', 'failed':'TERMINATE'}
        )
        smach.StateMachine.add(
            'LOCALIZE',
            LocalizeState(),
            transitions={'succeeded':'IDLE', 'failed':'TERMINATE'}
        )
        smach.StateMachine.add(
            'IDLE',
            IdleState(),
            transitions={'explore':'EXPLORE', 'comms':'COMMS', 'powerdown':'POWERDOWN'}
        )
        smach.StateMachine.add(
            'POWERDOWN',
            create_powerdown_sm(),
            transitions={'succeeded':'END'}
        )
        smach.StateMachine.add(
            'TERMINATE',
            PowerdownState(),
            transitions={'succeeded':'END'}
        )
        smach.StateMachine.add(
            'END',
            EndState(),
            transitions={'succeeded':'succeeded'}
        )
    return sm
