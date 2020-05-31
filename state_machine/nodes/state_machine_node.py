#!/usr/bin/env python

import rospy
import smach

def main():
    rospy.init_node('state_machine')

    # Create the state machine
    sm = smach.StateMachine(outcomes=['END'])

    # Add states to the state machine
    with sm:
        smach.StateMachine.add(
            'START',
            StartState(),
            transitions={'outcome1':'END'}
        )
        smach.StateMachine.add(
            'END',
            EndState(),
            transitions={'outcome1':'END'}
        )

    # Execute the state machine plan
    outcome = sm.execute()


if __name__ == '__main__':
    main()
