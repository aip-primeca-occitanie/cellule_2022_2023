cmake_minimum_required(VERSION 3.16.3)
project(robots)


add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  genmsg
  message_generation
  commande_locale
)

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   MoveRobot.msg
   FaireTacheMsg.msg
   FinDeplacerPiece_Msg.msg
   Msg_numrobot.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
 )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES robot
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
   CATKIN_DEPENDS message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(robot src/main_robot.cpp src/Robot.cpp src/Poste.cpp)
target_link_libraries(robot ${catkin_LIBRARIES})
add_dependencies(robot ${PROJECT_NAME}_gencpp ${catkin_EXPORTED_TARGETS})

