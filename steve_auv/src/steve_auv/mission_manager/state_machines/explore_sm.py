####################################
# 2019-2020, Lockheed Martin Space #
# TDC Team #2, Scuba Squad         #
####################################

import rospy
import smach
import steve_auv.mission_manager as mm


def build_explore_sm():
    """Builds the state machine for the explore sequence."""
    rospy.loginfo(f"Building 'EXPLORE' state machine")

    # Add states to the state machine
    sm = smach.StateMachine(outcomes=['succeeded', 'failed'])
    with sm:
        smach.StateMachine.add(
            'DIVE',
            SimpleActionState(
                gnc_topic,
                GncAction,
                goal=GncGoal('dive'),
                result_cb=action_cb,
                exec_timeout=rospy.Duration(60.0),
                server_wait_timeout=rospy.Duration(10.0)
            ),
            transitions={'succeeded':'EXPLORE', 'failed':'failed'}
        )
        smach.StateMachine.add(
            'EXPLORE',
            ExploreState(),
            transitions={'succeeded':'SURFACE', 'preempted':'SURFACE', 'failed':'SURFACE'}
        )
        smach.StateMachine.add(
            'SURFACE',
            SimpleActionState(
                gnc_topic,
                GncAction,
                goal=GncGoal('surface'),
                result_cb=action_cb,
                exec_timeout=rospy.Duration(60.0),
                server_wait_timeout=rospy.Duration(10.0)
            ),
            transitions={'succeeded':'succeeded', 'failed':'failed'}
        )
    return sm
