cmake_minimum_required(VERSION 3.0.2)
project(steve_auv)

find_package(catkin REQUIRED COMPONENTS
  rospy
  smach
  smach_msgs
  smach_ros
  std_msgs
)

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

catkin_add_nosetests(test/state_machine/test_mission_clock.py)
