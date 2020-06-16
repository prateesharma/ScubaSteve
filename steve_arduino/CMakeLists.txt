cmake_minimum_required(VERSION 3.0.2)
project(steve_arduino)

find_package(catkin REQUIRED COMPONENTS)
catkin_package(DEPENDS)

install(DIRECTORY src
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
