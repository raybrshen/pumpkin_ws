# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "analog_read: 2 messages, 0 services")

set(MSG_I_FLAGS "-Ianalog_read:/home/rafaelpaiva/workspace/pumpkin_ws/src/analog_read/msg;-Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(analog_read_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/rafaelpaiva/workspace/pumpkin_ws/src/analog_read/msg/analog_array.msg" NAME_WE)
add_custom_target(_analog_read_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "analog_read" "/home/rafaelpaiva/workspace/pumpkin_ws/src/analog_read/msg/analog_array.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/rafaelpaiva/workspace/pumpkin_ws/src/analog_read/msg/analog_fields.msg" NAME_WE)
add_custom_target(_analog_read_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "analog_read" "/home/rafaelpaiva/workspace/pumpkin_ws/src/analog_read/msg/analog_fields.msg" ""
)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(analog_read
  "/home/rafaelpaiva/workspace/pumpkin_ws/src/analog_read/msg/analog_array.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/analog_read
)
_generate_msg_cpp(analog_read
  "/home/rafaelpaiva/workspace/pumpkin_ws/src/analog_read/msg/analog_fields.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/analog_read
)

### Generating Services

### Generating Module File
_generate_module_cpp(analog_read
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/analog_read
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(analog_read_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(analog_read_generate_messages analog_read_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/rafaelpaiva/workspace/pumpkin_ws/src/analog_read/msg/analog_array.msg" NAME_WE)
add_dependencies(analog_read_generate_messages_cpp _analog_read_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rafaelpaiva/workspace/pumpkin_ws/src/analog_read/msg/analog_fields.msg" NAME_WE)
add_dependencies(analog_read_generate_messages_cpp _analog_read_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(analog_read_gencpp)
add_dependencies(analog_read_gencpp analog_read_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS analog_read_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(analog_read
  "/home/rafaelpaiva/workspace/pumpkin_ws/src/analog_read/msg/analog_array.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/analog_read
)
_generate_msg_lisp(analog_read
  "/home/rafaelpaiva/workspace/pumpkin_ws/src/analog_read/msg/analog_fields.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/analog_read
)

### Generating Services

### Generating Module File
_generate_module_lisp(analog_read
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/analog_read
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(analog_read_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(analog_read_generate_messages analog_read_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/rafaelpaiva/workspace/pumpkin_ws/src/analog_read/msg/analog_array.msg" NAME_WE)
add_dependencies(analog_read_generate_messages_lisp _analog_read_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rafaelpaiva/workspace/pumpkin_ws/src/analog_read/msg/analog_fields.msg" NAME_WE)
add_dependencies(analog_read_generate_messages_lisp _analog_read_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(analog_read_genlisp)
add_dependencies(analog_read_genlisp analog_read_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS analog_read_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(analog_read
  "/home/rafaelpaiva/workspace/pumpkin_ws/src/analog_read/msg/analog_array.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/analog_read
)
_generate_msg_py(analog_read
  "/home/rafaelpaiva/workspace/pumpkin_ws/src/analog_read/msg/analog_fields.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/analog_read
)

### Generating Services

### Generating Module File
_generate_module_py(analog_read
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/analog_read
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(analog_read_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(analog_read_generate_messages analog_read_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/rafaelpaiva/workspace/pumpkin_ws/src/analog_read/msg/analog_array.msg" NAME_WE)
add_dependencies(analog_read_generate_messages_py _analog_read_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rafaelpaiva/workspace/pumpkin_ws/src/analog_read/msg/analog_fields.msg" NAME_WE)
add_dependencies(analog_read_generate_messages_py _analog_read_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(analog_read_genpy)
add_dependencies(analog_read_genpy analog_read_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS analog_read_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/analog_read)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/analog_read
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(analog_read_generate_messages_cpp std_msgs_generate_messages_cpp)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/analog_read)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/analog_read
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(analog_read_generate_messages_lisp std_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/analog_read)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/analog_read\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/analog_read
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(analog_read_generate_messages_py std_msgs_generate_messages_py)
