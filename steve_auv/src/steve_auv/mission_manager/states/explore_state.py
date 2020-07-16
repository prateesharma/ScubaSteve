####################################
# 2019-2020, Lockheed Martin Space #
# TDC Team #2, Scuba Squad         #
####################################

import actionlib
import rospy


def explore_cb(userdata, status, result):
    """Handles logging and flags for the explore state."""
    if status == actionlib.GoalStatus.SUCCEEDED:
        rospy.loginfo("Explore goal completed successfully: continue")
    elif status == actionlib.GoalStatus.PREEMPTED:
        rospy.loginfo("Explore goal preempted successfully: continue")
    else:
        rospy.logerr("Something went wrong: failure, powering down")
        userdata.is_failed = True 