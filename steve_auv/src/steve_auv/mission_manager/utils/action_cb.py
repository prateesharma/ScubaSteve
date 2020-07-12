####################################
# 2019-2020, Lockheed Martin Space #
# TDC Team #2, Scuba Squad         #
####################################

import actionlib
import rospy


def action_cb(userdata, status, result):
    """Handles logging and flags for action states."""
    # If received, continue
    if status == actionlib.GoalStatus.SUCCEEDED:
        rospy.loginfo(f"Received signal: continuing")
        return 'succeeded'
    else:
        rospy.logerr(f"Goal timed out: failure, powering down")
        userdata.is_failed = True
        return 'failed'
