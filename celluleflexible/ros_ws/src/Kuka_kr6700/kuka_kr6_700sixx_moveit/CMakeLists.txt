cmake_minimum_required(VERSION 3.16.3)

project(kuka_kr6_700sixx_moveit)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY meshes urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
