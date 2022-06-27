cmake_minimum_required(VERSION 3.16.3)

project(staubli_tx2_60l_moveit)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY config launch meshes urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
