# Install script for directory: /home/projet-long20/TERcelluleflexible/celluleflexible/ros_ws/src/commande_locale

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/commande_locale/srv" TYPE FILE FILES
    "/home/projet-long20/TERcelluleflexible/celluleflexible/ros_ws/src/commande_locale/srv/SrvAddProduct.srv"
    "/home/projet-long20/TERcelluleflexible/celluleflexible/ros_ws/src/commande_locale/srv/SrvFinInit.srv"
    "/home/projet-long20/TERcelluleflexible/celluleflexible/ros_ws/src/commande_locale/srv/SrvAddProductPushBack.srv"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/commande_locale/msg" TYPE FILE FILES
    "/home/projet-long20/TERcelluleflexible/celluleflexible/ros_ws/src/commande_locale/msg/Msg_AddProduct.msg"
    "/home/projet-long20/TERcelluleflexible/celluleflexible/ros_ws/src/commande_locale/msg/Msg_StopControl.msg"
    "/home/projet-long20/TERcelluleflexible/celluleflexible/ros_ws/src/commande_locale/msg/Msg_Color.msg"
    "/home/projet-long20/TERcelluleflexible/celluleflexible/ros_ws/src/commande_locale/msg/Msg_SwitchControl.msg"
    "/home/projet-long20/TERcelluleflexible/celluleflexible/ros_ws/src/commande_locale/msg/Msg_PinControl.msg"
    "/home/projet-long20/TERcelluleflexible/celluleflexible/ros_ws/src/commande_locale/msg/Msg_AddProduct.msg"
    "/home/projet-long20/TERcelluleflexible/celluleflexible/ros_ws/src/commande_locale/msg/Msg_Erreur.msg"
    "/home/projet-long20/TERcelluleflexible/celluleflexible/ros_ws/src/commande_locale/msg/RobotJoints.msg"
    "/home/projet-long20/TERcelluleflexible/celluleflexible/ros_ws/src/commande_locale/msg/TacheFinieMsg.msg"
    "/home/projet-long20/TERcelluleflexible/celluleflexible/ros_ws/src/commande_locale/msg/DeplacerPieceMsg.msg"
    "/home/projet-long20/TERcelluleflexible/celluleflexible/ros_ws/src/commande_locale/msg/Actionneurs.msg"
    "/home/projet-long20/TERcelluleflexible/celluleflexible/ros_ws/src/commande_locale/msg/Capteurs.msg"
    "/home/projet-long20/TERcelluleflexible/celluleflexible/ros_ws/src/commande_locale/msg/Msg_ChoixMode.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/commande_locale/cmake" TYPE FILE FILES "/home/projet-long20/TERcelluleflexible/celluleflexible/ros_ws/build/commande_locale/catkin_generated/installspace/commande_locale-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/projet-long20/TERcelluleflexible/celluleflexible/ros_ws/devel/include/commande_locale")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/projet-long20/TERcelluleflexible/celluleflexible/ros_ws/devel/share/roseus/ros/commande_locale")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/projet-long20/TERcelluleflexible/celluleflexible/ros_ws/devel/share/common-lisp/ros/commande_locale")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/projet-long20/TERcelluleflexible/celluleflexible/ros_ws/devel/share/gennodejs/ros/commande_locale")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python3" -m compileall "/home/projet-long20/TERcelluleflexible/celluleflexible/ros_ws/devel/lib/python3/dist-packages/commande_locale")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/home/projet-long20/TERcelluleflexible/celluleflexible/ros_ws/devel/lib/python3/dist-packages/commande_locale")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/projet-long20/TERcelluleflexible/celluleflexible/ros_ws/build/commande_locale/catkin_generated/installspace/commande_locale.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/commande_locale/cmake" TYPE FILE FILES "/home/projet-long20/TERcelluleflexible/celluleflexible/ros_ws/build/commande_locale/catkin_generated/installspace/commande_locale-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/commande_locale/cmake" TYPE FILE FILES
    "/home/projet-long20/TERcelluleflexible/celluleflexible/ros_ws/build/commande_locale/catkin_generated/installspace/commande_localeConfig.cmake"
    "/home/projet-long20/TERcelluleflexible/celluleflexible/ros_ws/build/commande_locale/catkin_generated/installspace/commande_localeConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/commande_locale" TYPE FILE FILES "/home/projet-long20/TERcelluleflexible/celluleflexible/ros_ws/src/commande_locale/package.xml")
endif()

