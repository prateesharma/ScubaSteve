####################################
# 2019-2020, Lockheed Martin Space #
# TDC Team #2, Scuba Squad         #
####################################

import rospy
import smach
import smach_ros
import steve_auv.mission_manager as mm

from mm.state_machines.explore_sm import build_explore_demo_sm
from mm.states.comms_state import CommsState
from mm.states.end_state import EndState
from mm.states.idle_state import IdleState
from mm.states.powerdown_state import PowerdownState
from mm.states.powerup_state import PowerupState
from mm.states.release_state import release_cb
from mm.states.start_state import StartState
from steve_auv.msg import CommsAction, CommsGoal


def build_mission_manager_demo_sm(topics):
    rospy.loginfo("Building mission manager state machine")

    # Create a state machine and add userdata fields
    sm = smach.StateMachine(outcomes=['succeeded', 'failed'])
    sm.userdata.command = None
    sm.userdata.is_failed = False
    sm.userdata.image_count = 0
    sm.userdata.new_image_index = 0
    sm.userdata.new_image_count = 0

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
                topics.comms_mode_topic,
                CommsAction,
                goal=CommsGoal('release'),
                result_cb=release_cb,
                exec_timeout=rospy.Duration(1800.0),
                server_wait_timeout=rospy.Duration(10.0)
            ),
            transitions={
                'succeeded':'IDLE',
                'preempted':'POWERDOWN',
                'failed':'FAIL_POWERDOWN'
            }
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
            build_explore_demo_sm(userdata),
            transitions={
                'succeeded':'IDLE',
                'failed':'FAIL_POWERDOWN'
            }
        )
        smach.StateMachine.add(
            'COMMS',
            CommsState(
                topics.comms_mode_topic,
                topics.vision_mode_topic,
                preempt_timeout=rospy.Duration(10.0),
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
            'FAIL_POWERDOWN',
            PowerdownState(),
            transitions={'succeeded':'failed', 'failed':'failed'}
        )
    return sm
