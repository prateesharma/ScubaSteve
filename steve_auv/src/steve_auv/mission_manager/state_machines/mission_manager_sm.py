####################################
# 2019-2020, Lockheed Martin Space #
# TDC Team #2, Scuba Squad         #
####################################

import rospy
import smach
import smach_ros
import steve_auv.mission_manager as mm

from mm.state_machines.comms_sm import build_comms_sm
from mm.state_machines.explore_sm import build_explore_sm
from mm.states.end_state import EndState
from mm.states.idle_state import IdleState
from mm.states.powerdown_state import PowerdownState
from mm.states.powerup_state import PowerupState
from mm.states.release_state import release_cb
from mm.states.start_state import StartState
from mm.utils import action_cb
from steve_auv.msg import CommsAction, CommsGoal, GncAction, GncGoal


def build_mission_manager_sm(topics):
    rospy.loginfo("Building mission manager state machine")

    # Create a state machine and add userdata fields
    sm = smach.StateMachine(outcomes=['succeeded', 'failed'])
    sm.userdata.command = None
    sm.userdata.is_failed = False

    # Add states to the empty state machine
    with sm:
        smach.StateMachine.add(
            'POWERUP',
            PowerupState(),
            transitions={'succeeded':'RELEASE', 'failed':'FAIL_POWERDOWN'}
        )
        smach.StateMachine.add(
            'RELEASE',
            smach_ros.SimpleActionState(
                topics.comms_topic,
                CommsAction,
                goal=CommsGoal('release'),
                result_cb=release_cb,
                exec_timeout=rospy.Duration(1800.0),
                server_wait_timeout=rospy.Duration(10.0)
            ),
            transitions={'succeeded':'SPLASHDOWN', 'failed':'FAIL_POWERDOWN'}
        )
        smach.StateMachine.add(
            'SPLASHDOWN',
            smach_ros.SimpleActionState(
                topics.gnc_topic,
                GncAction,
                goal=GncGoal('splashdown'),
                result_cb=action_cb,
                exec_timeout=rospy.Duration(60.0),
                server_wait_timeout=rospy.Duration(10.0)
            ),
            transitions={'succeeded':'LOCALIZE', 'failed':'FAIL_POWERDOWN'}
        )
        smach.StateMachine.add(
            'LOCALIZE',
            smach_ros.SimpleActionState(
                topics.gnc_topic,
                GncAction,
                goal=GncGoal('localize'),
                result_cb=action_cb,
                exec_timeout=rospy.Duration(300.0),
                server_wait_timeout=rospy.Duration(10.0)
            ),
            transitions={'succeeded':'IDLE', 'failed':'FAIL_POWERDOWN'}
        )
        smach.StateMachine.add(
            'IDLE',
            IdleState(),
            transitions={
                'explore':'EXPLORE',
                'comms':'COMMS',
                'powerdown':'POWERDOWN',
                'failed':'FAIL_POWERDOWN'
            }
        )
        smach.StateMachine.add(
            'EXPLORE',
            build_explore_sm(userdata),
            transitions={
                'succeeded':'IDLE',
                'failed':'FAIL_POWERDOWN',
                'failed_underwater':'FAIL_SURFACE'
            }
        )
        smach.StateMachine.add(
            'COMMS',
            ScheduledActionState(
                topics.comms_topic,
                CommsAction,
                goal=GncGoal('comms'),
                result_cb=comms_cb,
                preempt_timeout=rospy.Duration(60.0),
                server_wait_timeout=rospy.Duration(10.0)
            ),
            transitions={
                'succeeded':'IDLE',
                'preempted':'IDLE',
                'failed':'FAIL_POWERDOWN'
            }
        )
        smach.StateMachine.add(
            'POWERDOWN',
            PowerdownState(),
            transitions={'succeeded':'succeeded', 'failed': 'failed'}
        )
        smach.StateMachine.add(
            'FAIL_SURFACE',
            smach_ros.SimpleActionState(
                topics.gnc_topic,
                GncAction,
                goal=GncGoal('surface'),
                result_cb=action_cb,
                exec_timeout=rospy.Duration(60.0),
                server_wait_timeout=rospy.Duration(10.0)
            ),
            transitions={
                'succeeded':'FAIL_POWERDOWN',
                'failed':'FAIL_POWERDOWN'
            }
        )
        smach.StateMachine.add(
            'FAIL_POWERDOWN',
            PowerdownState(),
            transitions={'succeeded':'failed', 'failed':'failed'}
        )
    return sm
