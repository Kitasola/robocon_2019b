# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "rc2019_commander: 1 messages, 0 services")

set(MSG_I_FLAGS "-Irc2019_commander:/home/tsuruhara/robocon_2019b/mr/src/rc2019_commander/msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(rc2019_commander_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/tsuruhara/robocon_2019b/mr/src/rc2019_commander/msg/button.msg" NAME_WE)
add_custom_target(_rc2019_commander_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "rc2019_commander" "/home/tsuruhara/robocon_2019b/mr/src/rc2019_commander/msg/button.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(rc2019_commander
  "/home/tsuruhara/robocon_2019b/mr/src/rc2019_commander/msg/button.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rc2019_commander
)

### Generating Services

### Generating Module File
_generate_module_cpp(rc2019_commander
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rc2019_commander
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(rc2019_commander_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(rc2019_commander_generate_messages rc2019_commander_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/tsuruhara/robocon_2019b/mr/src/rc2019_commander/msg/button.msg" NAME_WE)
add_dependencies(rc2019_commander_generate_messages_cpp _rc2019_commander_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(rc2019_commander_gencpp)
add_dependencies(rc2019_commander_gencpp rc2019_commander_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS rc2019_commander_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(rc2019_commander
  "/home/tsuruhara/robocon_2019b/mr/src/rc2019_commander/msg/button.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/rc2019_commander
)

### Generating Services

### Generating Module File
_generate_module_eus(rc2019_commander
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/rc2019_commander
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(rc2019_commander_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(rc2019_commander_generate_messages rc2019_commander_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/tsuruhara/robocon_2019b/mr/src/rc2019_commander/msg/button.msg" NAME_WE)
add_dependencies(rc2019_commander_generate_messages_eus _rc2019_commander_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(rc2019_commander_geneus)
add_dependencies(rc2019_commander_geneus rc2019_commander_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS rc2019_commander_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(rc2019_commander
  "/home/tsuruhara/robocon_2019b/mr/src/rc2019_commander/msg/button.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rc2019_commander
)

### Generating Services

### Generating Module File
_generate_module_lisp(rc2019_commander
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rc2019_commander
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(rc2019_commander_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(rc2019_commander_generate_messages rc2019_commander_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/tsuruhara/robocon_2019b/mr/src/rc2019_commander/msg/button.msg" NAME_WE)
add_dependencies(rc2019_commander_generate_messages_lisp _rc2019_commander_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(rc2019_commander_genlisp)
add_dependencies(rc2019_commander_genlisp rc2019_commander_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS rc2019_commander_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(rc2019_commander
  "/home/tsuruhara/robocon_2019b/mr/src/rc2019_commander/msg/button.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/rc2019_commander
)

### Generating Services

### Generating Module File
_generate_module_nodejs(rc2019_commander
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/rc2019_commander
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(rc2019_commander_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(rc2019_commander_generate_messages rc2019_commander_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/tsuruhara/robocon_2019b/mr/src/rc2019_commander/msg/button.msg" NAME_WE)
add_dependencies(rc2019_commander_generate_messages_nodejs _rc2019_commander_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(rc2019_commander_gennodejs)
add_dependencies(rc2019_commander_gennodejs rc2019_commander_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS rc2019_commander_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(rc2019_commander
  "/home/tsuruhara/robocon_2019b/mr/src/rc2019_commander/msg/button.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rc2019_commander
)

### Generating Services

### Generating Module File
_generate_module_py(rc2019_commander
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rc2019_commander
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(rc2019_commander_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(rc2019_commander_generate_messages rc2019_commander_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/tsuruhara/robocon_2019b/mr/src/rc2019_commander/msg/button.msg" NAME_WE)
add_dependencies(rc2019_commander_generate_messages_py _rc2019_commander_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(rc2019_commander_genpy)
add_dependencies(rc2019_commander_genpy rc2019_commander_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS rc2019_commander_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rc2019_commander)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rc2019_commander
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(rc2019_commander_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/rc2019_commander)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/rc2019_commander
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(rc2019_commander_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rc2019_commander)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rc2019_commander
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(rc2019_commander_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/rc2019_commander)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/rc2019_commander
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(rc2019_commander_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rc2019_commander)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rc2019_commander\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rc2019_commander
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(rc2019_commander_generate_messages_py std_msgs_generate_messages_py)
endif()
