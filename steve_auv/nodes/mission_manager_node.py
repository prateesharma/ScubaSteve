####################################
# 2019-2020, Lockheed Martin Space #
# TDC Team #2, Scuba Squad         #
####################################

#!/usr/bin/env python

import rospy
import steve_auv.mission_manager as mm

from mm.state_machines.mission_manager_sm import build_mission_manager_sm
from mm.utils.mission_clock import MissionClock
from mm.utils.topics import MissionManagerTopics


def main():
    rospy.init_node('mission_manager')

    # Add topics and flags to the empty object
    topics = MissionManagerTopics()
    topics.comms_mode_topic = rospy.get_param('~comms_mode_topic')
    topics.gnc_mode_topic = rospy.get_param('~gnc_mode_topic')

    # Configure the mission schedule
    schedule = [
        ( 0, 'EXPLORE'),
        (15, 'COMMS'),
        (20, 'EXPLORE'),
        (35, 'COMMS'),
        (40, 'EXPLORE'),
        (55, 'COMMS'),
        (60, 'POWERDOWN')
    ]
    mc = MissionClock.get_instance()
    mc.set_mission_schedule(schedule)

    # Create the state machine
    sm = build_mission_manager_sm(topics)

    # Execute the state machine plan
    rospy.loginfo("Executing mission manager")
    outcome = sm.execute()
    rospy.spin()


if __name__ == '__main__':
    main()
