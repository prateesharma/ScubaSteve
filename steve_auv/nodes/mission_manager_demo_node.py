####################################
# 2019-2020, Lockheed Martin Space #
# TDC Team #2, Scuba Squad         #
####################################

#!/usr/bin/env python

import rospy
import steve_auv.mission_manager as mm

from mm.state_machines.mission_manager_demo_sm import (
    build_mission_manager_demo_sm
)
from mm.utils.mission_clock import MissionClock
from mm.utils.topics import MissionManagerTopics


def main():
    rospy.init_node('mission_manager_demo')

    # Add topics and flags to the empty object
    topics = MissionManagerTopics()
    topics.comms_topic = rospy.get_param('~comms_topic')
    topics.gnc_topic = rospy.get_param('~gnc_topic')

    # Configure the mission schedule
    schedule = [
        ( 0, 'EXPLORE'),
        ( 3, 'COMMS'),
        ( 5, 'EXPLORE'),
        ( 8, 'COMMS'),
        ( 10, 'EXPLORE'),
        ( 13, 'COMMS'),
        ( 15, 'POWERDOWN')
    ]
    mc = MissionClock.get_instance()
    mc.set_mission_schedule(schedule)

    # Create the state machine
    sm = build_mission_manager_demo_sm(topics)

    # Execute the state machine plan
    rospy.loginfo("Executing mission manager")
    outcome = sm.execute()
    rospy.spin()


if __name__ == '__main__':
    main()
