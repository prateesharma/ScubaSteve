####################################
# 2019-2020, Lockheed Martin Space #
# TDC Team #2, Scuba Squad         #
####################################

import actionlib
import rospy
import steve_auv.mission_manager as mm

from mm.utils.mission_clock import MissionClock


def release_cb(userdata, status, result):
    """Handles logging and flags for the release state."""
    if status == actionlib.GoalStatus.SUCCEEDED:
        rospy.loginfo("Received signal: starting clock and continuing")
        mc = MissionClock.get_instance()
        mc.reset()
    else:
        rospy.logerr("Goal timed out: failure, powering down")
        userdata.is_failed = True
