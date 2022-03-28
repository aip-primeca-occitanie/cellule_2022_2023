# Install script for directory: /home/projet-long20/TERcelluleflexible/celluleflexible/ros_ws/src/shuttles

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/projet-long20/TERcelluleflexible/celluleflexible/ros_ws/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/shuttles/msg" TYPE FILE FILES
    "/home/projet-long20/TERcelluleflexible/celluleflexible/ros_ws/src/shuttles/msg/msgShuttleChange.msg"
    "/home/projet-long20/TERcelluleflexible/celluleflexible/ros_ws/src/shuttles/msg/msgShuttleCreate.msg"
    "/home/projet-long20/TERcelluleflexible/celluleflexible/ros_ws/src/shuttles/msg/Msg_Erreur.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/shuttles/srv" TYPE FILE FILES "/home/projet-long20/TERcelluleflexible/celluleflexible/ros_ws/src/shuttles/srv/shuttle_id.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/shuttles/cmake" TYPE FILE FILES "/home/projet-long20/TERcelluleflexible/celluleflexible/ros_ws/build/shuttles/catkin_generated/installspace/shuttles-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/projet-long20/TERcelluleflexible/celluleflexible/ros_ws/devel/include/shuttles")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/projet-long20/TERcelluleflexible/celluleflexible/ros_ws/devel/share/roseus/ros/shuttles")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/projet-long20/TERcelluleflexible/celluleflexible/ros_ws/devel/share/common-lisp/ros/shuttles")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/projet-long20/TERcelluleflexible/celluleflexible/ros_ws/devel/share/gennodejs/ros/shuttles")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python3" -m compileall "/home/projet-long20/TERcelluleflexible/celluleflexible/ros_ws/devel/lib/python3/dist-packages/shuttles")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/home/projet-long20/TERcelluleflexible/celluleflexible/ros_ws/devel/lib/python3/dist-packages/shuttles")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/projet-long20/TERcelluleflexible/celluleflexible/ros_ws/build/shuttles/catkin_generated/installspace/shuttles.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/shuttles/cmake" TYPE FILE FILES "/home/projet-long20/TERcelluleflexible/celluleflexible/ros_ws/build/shuttles/catkin_generated/installspace/shuttles-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/shuttles/cmake" TYPE FILE FILES
    "/home/projet-long20/TERcelluleflexible/celluleflexible/ros_ws/build/shuttles/catkin_generated/installspace/shuttlesConfig.cmake"
    "/home/projet-long20/TERcelluleflexible/celluleflexible/ros_ws/build/shuttles/catkin_generated/installspace/shuttlesConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/shuttles" TYPE FILE FILES "/home/projet-long20/TERcelluleflexible/celluleflexible/ros_ws/src/shuttles/package.xml")
endif()

