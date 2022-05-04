cmake_minimum_required(VERSION 3.16.3)
project(robotic_platform_moveit)

find_package(catkin REQUIRED COMPONENTS
roscpp
std_msgs
commande_locale)

include_directories(include ${catkin_INCLUDE_DIRS})

catkin_package(CATKIN_DEPENDS
roscpp
std_msgs
commande_locale)

add_executable(main_robotic_platform src/main_robotic_platform.cpp src/robotic_platform.cpp)

add_dependencies(main_robotic_platform ${${PROJECT_NAME}_gencpp} ${catkin_EXPORTED_TARGETS})

target_link_libraries(main_robotic_platform ${catkin_LIBRARIES} )

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
