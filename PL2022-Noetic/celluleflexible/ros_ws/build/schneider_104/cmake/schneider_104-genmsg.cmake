# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "schneider_104: 2 messages, 1 services")

set(MSG_I_FLAGS "-Ischneider_104:/home/projet-long20/TERcelluleflexible/celluleflexible/ros_ws/src/schneider_104/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(schneider_104_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/projet-long20/TERcelluleflexible/celluleflexible/ros_ws/src/schneider_104/msg/Control_cellule.msg" NAME_WE)
add_custom_target(_schneider_104_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "schneider_104" "/home/projet-long20/TERcelluleflexible/celluleflexible/ros_ws/src/schneider_104/msg/Control_cellule.msg" ""
)

get_filename_component(_filename "/home/projet-long20/TERcelluleflexible/celluleflexible/ros_ws/src/schneider_104/msg/Msg_SensorState.msg" NAME_WE)
add_custom_target(_schneider_104_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "schneider_104" "/home/projet-long20/TERcelluleflexible/celluleflexible/ros_ws/src/schneider_104/msg/Msg_SensorState.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/projet-long20/TERcelluleflexible/celluleflexible/ros_ws/src/schneider_104/srv/Retour_cellule_104.srv" NAME_WE)
add_custom_target(_schneider_104_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "schneider_104" "/home/projet-long20/TERcelluleflexible/celluleflexible/ros_ws/src/schneider_104/srv/Retour_cellule_104.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(schneider_104
  "/home/projet-long20/TERcelluleflexible/celluleflexible/ros_ws/src/schneider_104/msg/Control_cellule.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/schneider_104
)
_generate_msg_cpp(schneider_104
  "/home/projet-long20/TERcelluleflexible/celluleflexible/ros_ws/src/schneider_104/msg/Msg_SensorState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/schneider_104
)

### Generating Services
_generate_srv_cpp(schneider_104
  "/home/projet-long20/TERcelluleflexible/celluleflexible/ros_ws/src/schneider_104/srv/Retour_cellule_104.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/schneider_104
)

### Generating Module File
_generate_module_cpp(schneider_104
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/schneider_104
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(schneider_104_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(schneider_104_generate_messages schneider_104_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/projet-long20/TERcelluleflexible/celluleflexible/ros_ws/src/schneider_104/msg/Control_cellule.msg" NAME_WE)
add_dependencies(schneider_104_generate_messages_cpp _schneider_104_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/projet-long20/TERcelluleflexible/celluleflexible/ros_ws/src/schneider_104/msg/Msg_SensorState.msg" NAME_WE)
add_dependencies(schneider_104_generate_messages_cpp _schneider_104_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/projet-long20/TERcelluleflexible/celluleflexible/ros_ws/src/schneider_104/srv/Retour_cellule_104.srv" NAME_WE)
add_dependencies(schneider_104_generate_messages_cpp _schneider_104_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(schneider_104_gencpp)
add_dependencies(schneider_104_gencpp schneider_104_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS schneider_104_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(schneider_104
  "/home/projet-long20/TERcelluleflexible/celluleflexible/ros_ws/src/schneider_104/msg/Control_cellule.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/schneider_104
)
_generate_msg_eus(schneider_104
  "/home/projet-long20/TERcelluleflexible/celluleflexible/ros_ws/src/schneider_104/msg/Msg_SensorState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/schneider_104
)

### Generating Services
_generate_srv_eus(schneider_104
  "/home/projet-long20/TERcelluleflexible/celluleflexible/ros_ws/src/schneider_104/srv/Retour_cellule_104.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/schneider_104
)

### Generating Module File
_generate_module_eus(schneider_104
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/schneider_104
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(schneider_104_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(schneider_104_generate_messages schneider_104_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/projet-long20/TERcelluleflexible/celluleflexible/ros_ws/src/schneider_104/msg/Control_cellule.msg" NAME_WE)
add_dependencies(schneider_104_generate_messages_eus _schneider_104_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/projet-long20/TERcelluleflexible/celluleflexible/ros_ws/src/schneider_104/msg/Msg_SensorState.msg" NAME_WE)
add_dependencies(schneider_104_generate_messages_eus _schneider_104_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/projet-long20/TERcelluleflexible/celluleflexible/ros_ws/src/schneider_104/srv/Retour_cellule_104.srv" NAME_WE)
add_dependencies(schneider_104_generate_messages_eus _schneider_104_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(schneider_104_geneus)
add_dependencies(schneider_104_geneus schneider_104_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS schneider_104_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(schneider_104
  "/home/projet-long20/TERcelluleflexible/celluleflexible/ros_ws/src/schneider_104/msg/Control_cellule.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/schneider_104
)
_generate_msg_lisp(schneider_104
  "/home/projet-long20/TERcelluleflexible/celluleflexible/ros_ws/src/schneider_104/msg/Msg_SensorState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/schneider_104
)

### Generating Services
_generate_srv_lisp(schneider_104
  "/home/projet-long20/TERcelluleflexible/celluleflexible/ros_ws/src/schneider_104/srv/Retour_cellule_104.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/schneider_104
)

### Generating Module File
_generate_module_lisp(schneider_104
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/schneider_104
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(schneider_104_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(schneider_104_generate_messages schneider_104_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/projet-long20/TERcelluleflexible/celluleflexible/ros_ws/src/schneider_104/msg/Control_cellule.msg" NAME_WE)
add_dependencies(schneider_104_generate_messages_lisp _schneider_104_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/projet-long20/TERcelluleflexible/celluleflexible/ros_ws/src/schneider_104/msg/Msg_SensorState.msg" NAME_WE)
add_dependencies(schneider_104_generate_messages_lisp _schneider_104_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/projet-long20/TERcelluleflexible/celluleflexible/ros_ws/src/schneider_104/srv/Retour_cellule_104.srv" NAME_WE)
add_dependencies(schneider_104_generate_messages_lisp _schneider_104_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(schneider_104_genlisp)
add_dependencies(schneider_104_genlisp schneider_104_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS schneider_104_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(schneider_104
  "/home/projet-long20/TERcelluleflexible/celluleflexible/ros_ws/src/schneider_104/msg/Control_cellule.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/schneider_104
)
_generate_msg_nodejs(schneider_104
  "/home/projet-long20/TERcelluleflexible/celluleflexible/ros_ws/src/schneider_104/msg/Msg_SensorState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/schneider_104
)

### Generating Services
_generate_srv_nodejs(schneider_104
  "/home/projet-long20/TERcelluleflexible/celluleflexible/ros_ws/src/schneider_104/srv/Retour_cellule_104.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/schneider_104
)

### Generating Module File
_generate_module_nodejs(schneider_104
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/schneider_104
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(schneider_104_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(schneider_104_generate_messages schneider_104_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/projet-long20/TERcelluleflexible/celluleflexible/ros_ws/src/schneider_104/msg/Control_cellule.msg" NAME_WE)
add_dependencies(schneider_104_generate_messages_nodejs _schneider_104_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/projet-long20/TERcelluleflexible/celluleflexible/ros_ws/src/schneider_104/msg/Msg_SensorState.msg" NAME_WE)
add_dependencies(schneider_104_generate_messages_nodejs _schneider_104_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/projet-long20/TERcelluleflexible/celluleflexible/ros_ws/src/schneider_104/srv/Retour_cellule_104.srv" NAME_WE)
add_dependencies(schneider_104_generate_messages_nodejs _schneider_104_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(schneider_104_gennodejs)
add_dependencies(schneider_104_gennodejs schneider_104_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS schneider_104_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(schneider_104
  "/home/projet-long20/TERcelluleflexible/celluleflexible/ros_ws/src/schneider_104/msg/Control_cellule.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/schneider_104
)
_generate_msg_py(schneider_104
  "/home/projet-long20/TERcelluleflexible/celluleflexible/ros_ws/src/schneider_104/msg/Msg_SensorState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/schneider_104
)

### Generating Services
_generate_srv_py(schneider_104
  "/home/projet-long20/TERcelluleflexible/celluleflexible/ros_ws/src/schneider_104/srv/Retour_cellule_104.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/schneider_104
)

### Generating Module File
_generate_module_py(schneider_104
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/schneider_104
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(schneider_104_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(schneider_104_generate_messages schneider_104_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/projet-long20/TERcelluleflexible/celluleflexible/ros_ws/src/schneider_104/msg/Control_cellule.msg" NAME_WE)
add_dependencies(schneider_104_generate_messages_py _schneider_104_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/projet-long20/TERcelluleflexible/celluleflexible/ros_ws/src/schneider_104/msg/Msg_SensorState.msg" NAME_WE)
add_dependencies(schneider_104_generate_messages_py _schneider_104_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/projet-long20/TERcelluleflexible/celluleflexible/ros_ws/src/schneider_104/srv/Retour_cellule_104.srv" NAME_WE)
add_dependencies(schneider_104_generate_messages_py _schneider_104_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(schneider_104_genpy)
add_dependencies(schneider_104_genpy schneider_104_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS schneider_104_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/schneider_104)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/schneider_104
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(schneider_104_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/schneider_104)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/schneider_104
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(schneider_104_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/schneider_104)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/schneider_104
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(schneider_104_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/schneider_104)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/schneider_104
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(schneider_104_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/schneider_104)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/schneider_104\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/schneider_104
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(schneider_104_generate_messages_py std_msgs_generate_messages_py)
endif()
