####################################
# 2019-2020, Lockheed Martin Space #
# TDC Team #2, Scuba Squad         #
####################################

import actionlib
import rospy


def comms_cb(userdata, status, result):
    """Handles logging and flags for the comms state."""
    userdata.command = result.command
    if status == actionlib.GoalStatus.SUCCEEDED:
        rospy.loginfo("Comms goal completed successfully: continue")
    elif status == actionlib.GoalStatus.PREEMPTED:
        rospy.loginfo("Comms goal preempted successfully: continue")
    else:
        rospy.logerr("Something went wrong: failure, powering down")
        userdata.is_failed = True
