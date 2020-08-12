cmake_minimum_required(VERSION 3.0.2)
project(steve_auv)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  rospy
  smach
  smach_msgs
  smach_ros
  std_msgs
)
add_action_files(DIRECTORY action FILES
  Comms.action
  Gnc.action
)
generate_messages(DEPENDENCIES actionlib_msgs)

catkin_python_setup()
catkin_package(DEPENDS)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY src
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

catkin_add_nosetests(test/steve_auv/comms/test_comms.py)
catkin_add_nosetests(test/steve_auv/gnc/test_gnc_demo.py)
catkin_add_nosetests(test/steve_auv/mission_manager/test_mission_clock.py)
catkin_add_nosetests(test/steve_auv/mission_manager/test_mission_manager_demo.py)
