cmake_minimum_required(VERSION 3.16.3)

project(kuka_kr6_700sixx_moveit)

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

install(DIRECTORY meshes urdf config launch scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

catkin_install_python(
    PROGRAMS scripts/execute_trajectory_kuka.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )