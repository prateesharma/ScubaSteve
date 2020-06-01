####################################
# 2019-2020, Lockheed Martin Space #
# TDC Team #2, Scuba Squad         #
####################################

import rospy
import smach

from states.powerup_state import PowerupState
from states.release_state import ReleaseState
from states.splashdown_state import SplashdownState
from states.start_state import StartState


def steve_state_machine(name):
    rospy.loginfo(f"Building state machine '{name}'")

    # Add states to an empty state machine
    sm = smach.StateMachine(outcomes=['END'])
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
            transitions='outcome1':'COMMS', 'outcome2':'EXPLORE', 'outcome3':'POWERDOWN'}
        )
        smach.StateMachine.add(
            'COMMS',
            CommsStateMachine(),
            transitions={'outcome1':'IDLE'}
        )
        smach.StateMachine.add(
            'EXPLORE',
            ExploreStateMachine(),
            transitions={'outcome1':'IDLE'}
        )
        smach.StateMachine.add(
            'POWERDOWN',
            PowerdownStateMachine(),
            transitions={'outcome1':'END'}
        )
        smach.StateMachine.add(
            'END',
            EndState(),
            transitions={'outcome1':'END'}
        )
    return sm
