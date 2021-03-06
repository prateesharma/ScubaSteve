cmake_minimum_required(VERSION 3.0.2)
project(steve_auv)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  message_generation
  scikit-image
  rospy
  smach
  smach_msgs
  smach_ros
  std_msgs
)
add_action_files(DIRECTORY action FILES
  Comms.action
  Gnc.action
  Vision.action
)
add_message_files(FILES
  GncThrustersMsg.msg
)

catkin_python_setup()

generate_messages(DEPENDENCIES
  std_msgs
  actionlib_msgs
)

catkin_package(CATKIN_DEPENDS
  message_runtime
)

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
catkin_add_nosetests(test/steve_auv/vision/test_vision.py)
