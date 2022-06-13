cmake_minimum_required(VERSION 3.0.2)
project(yaskawa4)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  rospy
  commande_locale
)

include_directories(include ${catkin_INCLUDE_DIRS})

catkin_package(
  CATKIN_DEPENDS
  roscpp
  std_msgs
  rospy
)

if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch)
endif()

catkin_install_python(
  PROGRAMS scripts/execute_trajectory_yaska4.py scripts/plot.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
