cmake_minimum_required(VERSION 3.0.2)
project(hc10_ros)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf
  gazebo_ros
  rospy
  message_generation
  commande_locale
)

find_package(gazebo REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS})

catkin_package(
  # INCLUDE_DIRS include
  CATKIN_DEPENDS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf
  gazebo_ros
  message_runtime
  rospy
  DEPENDS GAZEBO
)


add_executable(main_yakuza src/main_yakuza.cpp src/Yakuza_tp.cpp)

add_dependencies(main_yakuza ${${PROJECT_NAME}_gencpp} ${catkin_EXPORTED_TARGETS})

target_link_libraries(main_yakuza ${catkin_LIBRARIES} )

if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch)
endif()

install(
  DIRECTORY
  #images
  meshes
  launch
  #rviz
  urdf
  config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(
  PROGRAMS
  scripts/add_interferences.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
