####################################
# 2019-2020, Lockheed Martin Space #
# TDC Team #2, Scuba Squad         #
####################################

import rospy
import smach
import steve_auv.mission_manager as mm

from mm.state_machines.comms_sm import build_comms_sm
from mm.state_machines.explore_sm import build_explore_sm
from mm.state_machines.powerdown_sm import build_powerdown_sm
from mm.states.end_state import EndState
from mm.states.idle_state import IdleState
from mm.states.powerdown_state import PowerdownState
from mm.states.powerup_state import PowerupState
from mm.states.release_state import release_cb
from mm.states.start_state import StartState
from mm.utils import action_cb
from steve_auv.msg import CommsAction, CommsGoal, GncAction, GncGoal


def build_mission_manager_sm(comms_topic, gnc_topic):
    rospy.loginfo(f"Building mission manager state machine")

    # Create a state machine and add userdata fields
    sm = smach.StateMachine(outcomes=['succeeded', 'failed'])
    sm.userdata.is_failed = False

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
            SimpleActionState(
                comms_topic,
                CommsAction,
                goal=CommsGoal('release'),
                result_cb=release_cb,
                exec_timeout=rospy.Duration(1800.0),
                server_wait_timeout=rospy.Duration(10.0)
            ),
            transitions={'suceeded':'SPLASHDOWN', 'failed':'TERMINATE'}
        )
        smach.StateMachine.add(
            'SPLASHDOWN',
            SimpleActionState(
                gnc_topic,
                GncAction,
                goal=GncGoal('splashdown'),
                result_cb=action_cb,
                exec_timeout=rospy.Duration(60.0),
                server_wait_timeout=rospy.Duration(10.0)
            ),
            transitions={'succeeded':'LOCALIZE', 'failed':'TERMINATE'}
        )
        smach.StateMachine.add(
            'LOCALIZE',
            SimpleActionState(
                gnc_topic,
                GncAction,
                goal=GncGoal('localize'),
                result_cb=action_cb,
                exec_timeout=rospy.Duration(300.0),
                server_wait_timeout=rospy.Duration(10.0)
            ),
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
            'EXPLORE',
            build_explore_sm(),
            transitions={'succeeded':'IDLE', 'failed':'POWERDOWN'}
        )
        smach.StateMachine.add(
            'COMMS',
            build_comms_sm(),
            transitions={'succeeded':'IDLE', 'failed':'POWERDOWN'}
        )
        smach.StateMachine.add(
            'POWERDOWN',
            build_powerdown_sm(),
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
