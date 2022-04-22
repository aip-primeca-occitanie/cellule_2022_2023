cmake_minimum_required(VERSION 3.0.2)
project(hc10_ros)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  rospy
  commande_locale
)

include_directories(include ${catkin_INCLUDE_DIRS})

catkin_package(
  # INCLUDE_DIRS include
  CATKIN_DEPENDS
  roscpp
  std_msgs
  rospy
)


add_executable(main_yakuza src/main_yakuza.cpp src/Yakuza_tp.cpp)

add_dependencies(main_yakuza ${${PROJECT_NAME}_gencpp} ${catkin_EXPORTED_TARGETS})

target_link_libraries(main_yakuza ${catkin_LIBRARIES} )

if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch)
endif()

catkin_install_python(
  PROGRAMS scripts/execute_trajectory.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
