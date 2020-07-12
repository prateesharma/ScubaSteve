####################################
# 2019-2020, Lockheed Martin Space #
# TDC Team #2, Scuba Squad         #
####################################

import rospy
import smach
import steve_auv.mission_manager as mm

from mm.state_machines.powerdown_sm import build_powerdown_sm
from mm.states.end_state import EndState
from mm.states.idle_state import IdleState
from mm.states.localize_state import LocalizeState
from mm.states.powerdown_state import PowerdownState
from mm.states.powerup_state import PowerupState
from mm.states.release_state import ReleaseState
from mm.states.splashdown_state import SplashdownState
from mm.states.start_state import StartState


def build_mission_manager_sm(release_topic, splashdown_topic, localize_topic):
    rospy.loginfo(f"Building mission manager state machine")

    # Create a state machine and add userdata fields
    sm = smach.StateMachine(outcomes=['succeeded', 'failed'])
    sm.userdata.is_failed = False
    sm.userdata.release_topic = release_topic
    sm.userdata.splashdown_topic = splashdown_topic
    sm.userdata.localize_topic = localize_topic

    # Add states to the empty state machine
    with sm:
        smach.StateMachine.add(
            'START',
            StartState(),
            transitions={'succeeded':'POWERUP'}
        )
        smach.StateMachine.add(
            'POWERUP',
            PowerupState(),
            transitions={'succeeded':'RELEASE', 'failed':'TERMINATE'}
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
            transitions={
                'explore':'EXPLORE',
                'comms':'COMMS',
                'powerdown':'POWERDOWN',
                'failed':'POWERDOWN'
            }
        )
        smach.StateMachine.add(
            'POWERDOWN',
            create_powerdown_sm(),
            transitions={'succeeded':'END'}
        )
        smach.StateMachine.add(
            'TERMINATE',
            PowerdownState(name='TERMINATE'),
            transitions={'succeeded':'END'}
        )
        smach.StateMachine.add(
            'END',
            EndState(),
            transitions={'succeeded':'succeeded', 'failed':'failed'}
        )
    return sm
