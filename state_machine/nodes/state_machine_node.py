#!/usr/bin/env python

import rospy
import smach


def steve_state_machine():
    # Create an empty state machine
    sm = smach.StateMachine(outcomes=['END'])

    # Add states to the state machine
    with sm:
        smach.StateMachine.add(
            'START',
            StartState(),
            transitions={'outcome1':'POWERUP'}
        )
        smach.StateMachine.add(
            'POWERUP',
            PowerUpState(),
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
            PowerDownStateMachine(),
            transitions={'outcome1':'END'}
        )
        smach.StateMachine.add(
            'END',
            EndState(),
            transitions={'outcome1':'END'}
        )
    return sm


def main():
    rospy.init_node('state_machine')

    # Create the state machine
    sm = steve_state_machine()

    # Execute the state machine plan
    outcome = sm.execute()


if __name__ == '__main__':
    main()
