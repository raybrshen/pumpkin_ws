# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "pumpkin_messages: 18 messages, 3 services")

set(MSG_I_FLAGS "-Ipumpkin_messages:/home/rafaelpaiva/workspace/pumpkin_ws/src/pumpkin_messages/msg;-Ipumpkin_messages:/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg;-Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg;-Iactionlib_msgs:/opt/ros/indigo/share/actionlib_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(pumpkin_messages_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/rafaelpaiva/workspace/pumpkin_ws/src/pumpkin_messages/msg/FileList.msg" NAME_WE)
add_custom_target(_pumpkin_messages_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "pumpkin_messages" "/home/rafaelpaiva/workspace/pumpkin_ws/src/pumpkin_messages/msg/FileList.msg" ""
)

get_filename_component(_filename "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/PlaybackGoal.msg" NAME_WE)
add_custom_target(_pumpkin_messages_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "pumpkin_messages" "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/PlaybackGoal.msg" ""
)

get_filename_component(_filename "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/PlaybackActionFeedback.msg" NAME_WE)
add_custom_target(_pumpkin_messages_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "pumpkin_messages" "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/PlaybackActionFeedback.msg" "actionlib_msgs/GoalStatus:actionlib_msgs/GoalID:std_msgs/Header:pumpkin_messages/PlaybackFeedback"
)

get_filename_component(_filename "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/RecordActionFeedback.msg" NAME_WE)
add_custom_target(_pumpkin_messages_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "pumpkin_messages" "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/RecordActionFeedback.msg" "actionlib_msgs/GoalStatus:actionlib_msgs/GoalID:std_msgs/Header:pumpkin_messages/RecordFeedback"
)

get_filename_component(_filename "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/RecordActionResult.msg" NAME_WE)
add_custom_target(_pumpkin_messages_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "pumpkin_messages" "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/RecordActionResult.msg" "actionlib_msgs/GoalStatus:actionlib_msgs/GoalID:std_msgs/Header:pumpkin_messages/RecordResult"
)

get_filename_component(_filename "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/PlaybackActionResult.msg" NAME_WE)
add_custom_target(_pumpkin_messages_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "pumpkin_messages" "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/PlaybackActionResult.msg" "actionlib_msgs/GoalStatus:actionlib_msgs/GoalID:std_msgs/Header:pumpkin_messages/PlaybackResult"
)

get_filename_component(_filename "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/RecordAction.msg" NAME_WE)
add_custom_target(_pumpkin_messages_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "pumpkin_messages" "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/RecordAction.msg" "pumpkin_messages/RecordActionResult:pumpkin_messages/RecordFeedback:actionlib_msgs/GoalStatus:actionlib_msgs/GoalID:pumpkin_messages/RecordActionFeedback:std_msgs/Header:pumpkin_messages/RecordGoal:pumpkin_messages/RecordResult:pumpkin_messages/RecordActionGoal"
)

get_filename_component(_filename "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/RecordFeedback.msg" NAME_WE)
add_custom_target(_pumpkin_messages_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "pumpkin_messages" "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/RecordFeedback.msg" ""
)

get_filename_component(_filename "/home/rafaelpaiva/workspace/pumpkin_ws/src/pumpkin_messages/msg/analog_array.msg" NAME_WE)
add_custom_target(_pumpkin_messages_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "pumpkin_messages" "/home/rafaelpaiva/workspace/pumpkin_ws/src/pumpkin_messages/msg/analog_array.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/PlaybackActionGoal.msg" NAME_WE)
add_custom_target(_pumpkin_messages_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "pumpkin_messages" "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/PlaybackActionGoal.msg" "actionlib_msgs/GoalID:std_msgs/Header:pumpkin_messages/PlaybackGoal"
)

get_filename_component(_filename "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/RecordActionGoal.msg" NAME_WE)
add_custom_target(_pumpkin_messages_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "pumpkin_messages" "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/RecordActionGoal.msg" "actionlib_msgs/GoalID:std_msgs/Header:pumpkin_messages/RecordGoal"
)

get_filename_component(_filename "/home/rafaelpaiva/workspace/pumpkin_ws/src/pumpkin_messages/srv/SSCMoveCommand.srv" NAME_WE)
add_custom_target(_pumpkin_messages_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "pumpkin_messages" "/home/rafaelpaiva/workspace/pumpkin_ws/src/pumpkin_messages/srv/SSCMoveCommand.srv" "pumpkin_messages/SSCMove:pumpkin_messages/SSCMoveList"
)

get_filename_component(_filename "/home/rafaelpaiva/workspace/pumpkin_ws/src/pumpkin_messages/srv/Serials.srv" NAME_WE)
add_custom_target(_pumpkin_messages_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "pumpkin_messages" "/home/rafaelpaiva/workspace/pumpkin_ws/src/pumpkin_messages/srv/Serials.srv" ""
)

get_filename_component(_filename "/home/rafaelpaiva/workspace/pumpkin_ws/src/pumpkin_messages/msg/SSCMove.msg" NAME_WE)
add_custom_target(_pumpkin_messages_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "pumpkin_messages" "/home/rafaelpaiva/workspace/pumpkin_ws/src/pumpkin_messages/msg/SSCMove.msg" ""
)

get_filename_component(_filename "/home/rafaelpaiva/workspace/pumpkin_ws/src/pumpkin_messages/srv/Files.srv" NAME_WE)
add_custom_target(_pumpkin_messages_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "pumpkin_messages" "/home/rafaelpaiva/workspace/pumpkin_ws/src/pumpkin_messages/srv/Files.srv" "pumpkin_messages/FileList"
)

get_filename_component(_filename "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/PlaybackFeedback.msg" NAME_WE)
add_custom_target(_pumpkin_messages_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "pumpkin_messages" "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/PlaybackFeedback.msg" ""
)

get_filename_component(_filename "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/PlaybackAction.msg" NAME_WE)
add_custom_target(_pumpkin_messages_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "pumpkin_messages" "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/PlaybackAction.msg" "pumpkin_messages/PlaybackFeedback:pumpkin_messages/PlaybackActionResult:actionlib_msgs/GoalID:pumpkin_messages/PlaybackGoal:pumpkin_messages/PlaybackActionGoal:pumpkin_messages/PlaybackResult:std_msgs/Header:actionlib_msgs/GoalStatus:pumpkin_messages/PlaybackActionFeedback"
)

get_filename_component(_filename "/home/rafaelpaiva/workspace/pumpkin_ws/src/pumpkin_messages/msg/SSCMoveList.msg" NAME_WE)
add_custom_target(_pumpkin_messages_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "pumpkin_messages" "/home/rafaelpaiva/workspace/pumpkin_ws/src/pumpkin_messages/msg/SSCMoveList.msg" "pumpkin_messages/SSCMove"
)

get_filename_component(_filename "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/RecordGoal.msg" NAME_WE)
add_custom_target(_pumpkin_messages_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "pumpkin_messages" "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/RecordGoal.msg" ""
)

get_filename_component(_filename "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/PlaybackResult.msg" NAME_WE)
add_custom_target(_pumpkin_messages_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "pumpkin_messages" "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/PlaybackResult.msg" ""
)

get_filename_component(_filename "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/RecordResult.msg" NAME_WE)
add_custom_target(_pumpkin_messages_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "pumpkin_messages" "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/RecordResult.msg" ""
)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(pumpkin_messages
  "/home/rafaelpaiva/workspace/pumpkin_ws/src/pumpkin_messages/msg/FileList.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pumpkin_messages
)
_generate_msg_cpp(pumpkin_messages
  "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/PlaybackGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pumpkin_messages
)
_generate_msg_cpp(pumpkin_messages
  "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/PlaybackActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/PlaybackFeedback.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pumpkin_messages
)
_generate_msg_cpp(pumpkin_messages
  "/home/rafaelpaiva/workspace/pumpkin_ws/src/pumpkin_messages/msg/SSCMoveList.msg"
  "${MSG_I_FLAGS}"
  "/home/rafaelpaiva/workspace/pumpkin_ws/src/pumpkin_messages/msg/SSCMove.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pumpkin_messages
)
_generate_msg_cpp(pumpkin_messages
  "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/RecordResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pumpkin_messages
)
_generate_msg_cpp(pumpkin_messages
  "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/RecordActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/RecordResult.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pumpkin_messages
)
_generate_msg_cpp(pumpkin_messages
  "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/PlaybackActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/PlaybackResult.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pumpkin_messages
)
_generate_msg_cpp(pumpkin_messages
  "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/RecordAction.msg"
  "${MSG_I_FLAGS}"
  "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/RecordActionResult.msg;/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/RecordFeedback.msg;/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/RecordActionFeedback.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/RecordGoal.msg;/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/RecordResult.msg;/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/RecordActionGoal.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pumpkin_messages
)
_generate_msg_cpp(pumpkin_messages
  "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/RecordFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pumpkin_messages
)
_generate_msg_cpp(pumpkin_messages
  "/home/rafaelpaiva/workspace/pumpkin_ws/src/pumpkin_messages/msg/analog_array.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pumpkin_messages
)
_generate_msg_cpp(pumpkin_messages
  "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/PlaybackActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/PlaybackGoal.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pumpkin_messages
)
_generate_msg_cpp(pumpkin_messages
  "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/RecordActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/RecordGoal.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pumpkin_messages
)
_generate_msg_cpp(pumpkin_messages
  "/home/rafaelpaiva/workspace/pumpkin_ws/src/pumpkin_messages/msg/SSCMove.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pumpkin_messages
)
_generate_msg_cpp(pumpkin_messages
  "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/PlaybackFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pumpkin_messages
)
_generate_msg_cpp(pumpkin_messages
  "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/PlaybackAction.msg"
  "${MSG_I_FLAGS}"
  "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/PlaybackFeedback.msg;/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/PlaybackActionResult.msg;/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/PlaybackGoal.msg;/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/PlaybackActionGoal.msg;/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/PlaybackResult.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/PlaybackActionFeedback.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pumpkin_messages
)
_generate_msg_cpp(pumpkin_messages
  "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/RecordGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pumpkin_messages
)
_generate_msg_cpp(pumpkin_messages
  "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/PlaybackResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pumpkin_messages
)
_generate_msg_cpp(pumpkin_messages
  "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/RecordActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/RecordFeedback.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pumpkin_messages
)

### Generating Services
_generate_srv_cpp(pumpkin_messages
  "/home/rafaelpaiva/workspace/pumpkin_ws/src/pumpkin_messages/srv/Serials.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pumpkin_messages
)
_generate_srv_cpp(pumpkin_messages
  "/home/rafaelpaiva/workspace/pumpkin_ws/src/pumpkin_messages/srv/Files.srv"
  "${MSG_I_FLAGS}"
  "/home/rafaelpaiva/workspace/pumpkin_ws/src/pumpkin_messages/msg/FileList.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pumpkin_messages
)
_generate_srv_cpp(pumpkin_messages
  "/home/rafaelpaiva/workspace/pumpkin_ws/src/pumpkin_messages/srv/SSCMoveCommand.srv"
  "${MSG_I_FLAGS}"
  "/home/rafaelpaiva/workspace/pumpkin_ws/src/pumpkin_messages/msg/SSCMove.msg;/home/rafaelpaiva/workspace/pumpkin_ws/src/pumpkin_messages/msg/SSCMoveList.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pumpkin_messages
)

### Generating Module File
_generate_module_cpp(pumpkin_messages
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pumpkin_messages
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(pumpkin_messages_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(pumpkin_messages_generate_messages pumpkin_messages_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/rafaelpaiva/workspace/pumpkin_ws/src/pumpkin_messages/msg/FileList.msg" NAME_WE)
add_dependencies(pumpkin_messages_generate_messages_cpp _pumpkin_messages_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/PlaybackGoal.msg" NAME_WE)
add_dependencies(pumpkin_messages_generate_messages_cpp _pumpkin_messages_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/PlaybackActionFeedback.msg" NAME_WE)
add_dependencies(pumpkin_messages_generate_messages_cpp _pumpkin_messages_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/RecordActionFeedback.msg" NAME_WE)
add_dependencies(pumpkin_messages_generate_messages_cpp _pumpkin_messages_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/RecordActionResult.msg" NAME_WE)
add_dependencies(pumpkin_messages_generate_messages_cpp _pumpkin_messages_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/PlaybackActionResult.msg" NAME_WE)
add_dependencies(pumpkin_messages_generate_messages_cpp _pumpkin_messages_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/RecordAction.msg" NAME_WE)
add_dependencies(pumpkin_messages_generate_messages_cpp _pumpkin_messages_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/RecordFeedback.msg" NAME_WE)
add_dependencies(pumpkin_messages_generate_messages_cpp _pumpkin_messages_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rafaelpaiva/workspace/pumpkin_ws/src/pumpkin_messages/msg/analog_array.msg" NAME_WE)
add_dependencies(pumpkin_messages_generate_messages_cpp _pumpkin_messages_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/PlaybackActionGoal.msg" NAME_WE)
add_dependencies(pumpkin_messages_generate_messages_cpp _pumpkin_messages_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/RecordActionGoal.msg" NAME_WE)
add_dependencies(pumpkin_messages_generate_messages_cpp _pumpkin_messages_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rafaelpaiva/workspace/pumpkin_ws/src/pumpkin_messages/srv/SSCMoveCommand.srv" NAME_WE)
add_dependencies(pumpkin_messages_generate_messages_cpp _pumpkin_messages_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rafaelpaiva/workspace/pumpkin_ws/src/pumpkin_messages/srv/Serials.srv" NAME_WE)
add_dependencies(pumpkin_messages_generate_messages_cpp _pumpkin_messages_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rafaelpaiva/workspace/pumpkin_ws/src/pumpkin_messages/msg/SSCMove.msg" NAME_WE)
add_dependencies(pumpkin_messages_generate_messages_cpp _pumpkin_messages_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rafaelpaiva/workspace/pumpkin_ws/src/pumpkin_messages/srv/Files.srv" NAME_WE)
add_dependencies(pumpkin_messages_generate_messages_cpp _pumpkin_messages_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/PlaybackFeedback.msg" NAME_WE)
add_dependencies(pumpkin_messages_generate_messages_cpp _pumpkin_messages_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/PlaybackAction.msg" NAME_WE)
add_dependencies(pumpkin_messages_generate_messages_cpp _pumpkin_messages_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rafaelpaiva/workspace/pumpkin_ws/src/pumpkin_messages/msg/SSCMoveList.msg" NAME_WE)
add_dependencies(pumpkin_messages_generate_messages_cpp _pumpkin_messages_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/RecordGoal.msg" NAME_WE)
add_dependencies(pumpkin_messages_generate_messages_cpp _pumpkin_messages_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/PlaybackResult.msg" NAME_WE)
add_dependencies(pumpkin_messages_generate_messages_cpp _pumpkin_messages_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/RecordResult.msg" NAME_WE)
add_dependencies(pumpkin_messages_generate_messages_cpp _pumpkin_messages_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(pumpkin_messages_gencpp)
add_dependencies(pumpkin_messages_gencpp pumpkin_messages_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS pumpkin_messages_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(pumpkin_messages
  "/home/rafaelpaiva/workspace/pumpkin_ws/src/pumpkin_messages/msg/FileList.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pumpkin_messages
)
_generate_msg_lisp(pumpkin_messages
  "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/PlaybackGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pumpkin_messages
)
_generate_msg_lisp(pumpkin_messages
  "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/PlaybackActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/PlaybackFeedback.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pumpkin_messages
)
_generate_msg_lisp(pumpkin_messages
  "/home/rafaelpaiva/workspace/pumpkin_ws/src/pumpkin_messages/msg/SSCMoveList.msg"
  "${MSG_I_FLAGS}"
  "/home/rafaelpaiva/workspace/pumpkin_ws/src/pumpkin_messages/msg/SSCMove.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pumpkin_messages
)
_generate_msg_lisp(pumpkin_messages
  "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/RecordResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pumpkin_messages
)
_generate_msg_lisp(pumpkin_messages
  "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/RecordActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/RecordResult.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pumpkin_messages
)
_generate_msg_lisp(pumpkin_messages
  "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/PlaybackActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/PlaybackResult.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pumpkin_messages
)
_generate_msg_lisp(pumpkin_messages
  "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/RecordAction.msg"
  "${MSG_I_FLAGS}"
  "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/RecordActionResult.msg;/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/RecordFeedback.msg;/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/RecordActionFeedback.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/RecordGoal.msg;/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/RecordResult.msg;/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/RecordActionGoal.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pumpkin_messages
)
_generate_msg_lisp(pumpkin_messages
  "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/RecordFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pumpkin_messages
)
_generate_msg_lisp(pumpkin_messages
  "/home/rafaelpaiva/workspace/pumpkin_ws/src/pumpkin_messages/msg/analog_array.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pumpkin_messages
)
_generate_msg_lisp(pumpkin_messages
  "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/PlaybackActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/PlaybackGoal.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pumpkin_messages
)
_generate_msg_lisp(pumpkin_messages
  "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/RecordActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/RecordGoal.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pumpkin_messages
)
_generate_msg_lisp(pumpkin_messages
  "/home/rafaelpaiva/workspace/pumpkin_ws/src/pumpkin_messages/msg/SSCMove.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pumpkin_messages
)
_generate_msg_lisp(pumpkin_messages
  "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/PlaybackFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pumpkin_messages
)
_generate_msg_lisp(pumpkin_messages
  "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/PlaybackAction.msg"
  "${MSG_I_FLAGS}"
  "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/PlaybackFeedback.msg;/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/PlaybackActionResult.msg;/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/PlaybackGoal.msg;/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/PlaybackActionGoal.msg;/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/PlaybackResult.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/PlaybackActionFeedback.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pumpkin_messages
)
_generate_msg_lisp(pumpkin_messages
  "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/RecordGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pumpkin_messages
)
_generate_msg_lisp(pumpkin_messages
  "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/PlaybackResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pumpkin_messages
)
_generate_msg_lisp(pumpkin_messages
  "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/RecordActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/RecordFeedback.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pumpkin_messages
)

### Generating Services
_generate_srv_lisp(pumpkin_messages
  "/home/rafaelpaiva/workspace/pumpkin_ws/src/pumpkin_messages/srv/Serials.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pumpkin_messages
)
_generate_srv_lisp(pumpkin_messages
  "/home/rafaelpaiva/workspace/pumpkin_ws/src/pumpkin_messages/srv/Files.srv"
  "${MSG_I_FLAGS}"
  "/home/rafaelpaiva/workspace/pumpkin_ws/src/pumpkin_messages/msg/FileList.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pumpkin_messages
)
_generate_srv_lisp(pumpkin_messages
  "/home/rafaelpaiva/workspace/pumpkin_ws/src/pumpkin_messages/srv/SSCMoveCommand.srv"
  "${MSG_I_FLAGS}"
  "/home/rafaelpaiva/workspace/pumpkin_ws/src/pumpkin_messages/msg/SSCMove.msg;/home/rafaelpaiva/workspace/pumpkin_ws/src/pumpkin_messages/msg/SSCMoveList.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pumpkin_messages
)

### Generating Module File
_generate_module_lisp(pumpkin_messages
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pumpkin_messages
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(pumpkin_messages_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(pumpkin_messages_generate_messages pumpkin_messages_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/rafaelpaiva/workspace/pumpkin_ws/src/pumpkin_messages/msg/FileList.msg" NAME_WE)
add_dependencies(pumpkin_messages_generate_messages_lisp _pumpkin_messages_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/PlaybackGoal.msg" NAME_WE)
add_dependencies(pumpkin_messages_generate_messages_lisp _pumpkin_messages_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/PlaybackActionFeedback.msg" NAME_WE)
add_dependencies(pumpkin_messages_generate_messages_lisp _pumpkin_messages_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/RecordActionFeedback.msg" NAME_WE)
add_dependencies(pumpkin_messages_generate_messages_lisp _pumpkin_messages_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/RecordActionResult.msg" NAME_WE)
add_dependencies(pumpkin_messages_generate_messages_lisp _pumpkin_messages_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/PlaybackActionResult.msg" NAME_WE)
add_dependencies(pumpkin_messages_generate_messages_lisp _pumpkin_messages_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/RecordAction.msg" NAME_WE)
add_dependencies(pumpkin_messages_generate_messages_lisp _pumpkin_messages_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/RecordFeedback.msg" NAME_WE)
add_dependencies(pumpkin_messages_generate_messages_lisp _pumpkin_messages_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rafaelpaiva/workspace/pumpkin_ws/src/pumpkin_messages/msg/analog_array.msg" NAME_WE)
add_dependencies(pumpkin_messages_generate_messages_lisp _pumpkin_messages_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/PlaybackActionGoal.msg" NAME_WE)
add_dependencies(pumpkin_messages_generate_messages_lisp _pumpkin_messages_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/RecordActionGoal.msg" NAME_WE)
add_dependencies(pumpkin_messages_generate_messages_lisp _pumpkin_messages_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rafaelpaiva/workspace/pumpkin_ws/src/pumpkin_messages/srv/SSCMoveCommand.srv" NAME_WE)
add_dependencies(pumpkin_messages_generate_messages_lisp _pumpkin_messages_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rafaelpaiva/workspace/pumpkin_ws/src/pumpkin_messages/srv/Serials.srv" NAME_WE)
add_dependencies(pumpkin_messages_generate_messages_lisp _pumpkin_messages_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rafaelpaiva/workspace/pumpkin_ws/src/pumpkin_messages/msg/SSCMove.msg" NAME_WE)
add_dependencies(pumpkin_messages_generate_messages_lisp _pumpkin_messages_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rafaelpaiva/workspace/pumpkin_ws/src/pumpkin_messages/srv/Files.srv" NAME_WE)
add_dependencies(pumpkin_messages_generate_messages_lisp _pumpkin_messages_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/PlaybackFeedback.msg" NAME_WE)
add_dependencies(pumpkin_messages_generate_messages_lisp _pumpkin_messages_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/PlaybackAction.msg" NAME_WE)
add_dependencies(pumpkin_messages_generate_messages_lisp _pumpkin_messages_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rafaelpaiva/workspace/pumpkin_ws/src/pumpkin_messages/msg/SSCMoveList.msg" NAME_WE)
add_dependencies(pumpkin_messages_generate_messages_lisp _pumpkin_messages_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/RecordGoal.msg" NAME_WE)
add_dependencies(pumpkin_messages_generate_messages_lisp _pumpkin_messages_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/PlaybackResult.msg" NAME_WE)
add_dependencies(pumpkin_messages_generate_messages_lisp _pumpkin_messages_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/RecordResult.msg" NAME_WE)
add_dependencies(pumpkin_messages_generate_messages_lisp _pumpkin_messages_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(pumpkin_messages_genlisp)
add_dependencies(pumpkin_messages_genlisp pumpkin_messages_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS pumpkin_messages_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(pumpkin_messages
  "/home/rafaelpaiva/workspace/pumpkin_ws/src/pumpkin_messages/msg/FileList.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pumpkin_messages
)
_generate_msg_py(pumpkin_messages
  "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/PlaybackGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pumpkin_messages
)
_generate_msg_py(pumpkin_messages
  "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/PlaybackActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/PlaybackFeedback.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pumpkin_messages
)
_generate_msg_py(pumpkin_messages
  "/home/rafaelpaiva/workspace/pumpkin_ws/src/pumpkin_messages/msg/SSCMoveList.msg"
  "${MSG_I_FLAGS}"
  "/home/rafaelpaiva/workspace/pumpkin_ws/src/pumpkin_messages/msg/SSCMove.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pumpkin_messages
)
_generate_msg_py(pumpkin_messages
  "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/RecordResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pumpkin_messages
)
_generate_msg_py(pumpkin_messages
  "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/RecordActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/RecordResult.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pumpkin_messages
)
_generate_msg_py(pumpkin_messages
  "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/PlaybackActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/PlaybackResult.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pumpkin_messages
)
_generate_msg_py(pumpkin_messages
  "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/RecordAction.msg"
  "${MSG_I_FLAGS}"
  "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/RecordActionResult.msg;/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/RecordFeedback.msg;/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/RecordActionFeedback.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/RecordGoal.msg;/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/RecordResult.msg;/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/RecordActionGoal.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pumpkin_messages
)
_generate_msg_py(pumpkin_messages
  "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/RecordFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pumpkin_messages
)
_generate_msg_py(pumpkin_messages
  "/home/rafaelpaiva/workspace/pumpkin_ws/src/pumpkin_messages/msg/analog_array.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pumpkin_messages
)
_generate_msg_py(pumpkin_messages
  "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/PlaybackActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/PlaybackGoal.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pumpkin_messages
)
_generate_msg_py(pumpkin_messages
  "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/RecordActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/RecordGoal.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pumpkin_messages
)
_generate_msg_py(pumpkin_messages
  "/home/rafaelpaiva/workspace/pumpkin_ws/src/pumpkin_messages/msg/SSCMove.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pumpkin_messages
)
_generate_msg_py(pumpkin_messages
  "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/PlaybackFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pumpkin_messages
)
_generate_msg_py(pumpkin_messages
  "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/PlaybackAction.msg"
  "${MSG_I_FLAGS}"
  "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/PlaybackFeedback.msg;/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/PlaybackActionResult.msg;/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/PlaybackGoal.msg;/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/PlaybackActionGoal.msg;/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/PlaybackResult.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/PlaybackActionFeedback.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pumpkin_messages
)
_generate_msg_py(pumpkin_messages
  "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/RecordGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pumpkin_messages
)
_generate_msg_py(pumpkin_messages
  "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/PlaybackResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pumpkin_messages
)
_generate_msg_py(pumpkin_messages
  "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/RecordActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/RecordFeedback.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pumpkin_messages
)

### Generating Services
_generate_srv_py(pumpkin_messages
  "/home/rafaelpaiva/workspace/pumpkin_ws/src/pumpkin_messages/srv/Serials.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pumpkin_messages
)
_generate_srv_py(pumpkin_messages
  "/home/rafaelpaiva/workspace/pumpkin_ws/src/pumpkin_messages/srv/Files.srv"
  "${MSG_I_FLAGS}"
  "/home/rafaelpaiva/workspace/pumpkin_ws/src/pumpkin_messages/msg/FileList.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pumpkin_messages
)
_generate_srv_py(pumpkin_messages
  "/home/rafaelpaiva/workspace/pumpkin_ws/src/pumpkin_messages/srv/SSCMoveCommand.srv"
  "${MSG_I_FLAGS}"
  "/home/rafaelpaiva/workspace/pumpkin_ws/src/pumpkin_messages/msg/SSCMove.msg;/home/rafaelpaiva/workspace/pumpkin_ws/src/pumpkin_messages/msg/SSCMoveList.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pumpkin_messages
)

### Generating Module File
_generate_module_py(pumpkin_messages
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pumpkin_messages
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(pumpkin_messages_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(pumpkin_messages_generate_messages pumpkin_messages_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/rafaelpaiva/workspace/pumpkin_ws/src/pumpkin_messages/msg/FileList.msg" NAME_WE)
add_dependencies(pumpkin_messages_generate_messages_py _pumpkin_messages_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/PlaybackGoal.msg" NAME_WE)
add_dependencies(pumpkin_messages_generate_messages_py _pumpkin_messages_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/PlaybackActionFeedback.msg" NAME_WE)
add_dependencies(pumpkin_messages_generate_messages_py _pumpkin_messages_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/RecordActionFeedback.msg" NAME_WE)
add_dependencies(pumpkin_messages_generate_messages_py _pumpkin_messages_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/RecordActionResult.msg" NAME_WE)
add_dependencies(pumpkin_messages_generate_messages_py _pumpkin_messages_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/PlaybackActionResult.msg" NAME_WE)
add_dependencies(pumpkin_messages_generate_messages_py _pumpkin_messages_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/RecordAction.msg" NAME_WE)
add_dependencies(pumpkin_messages_generate_messages_py _pumpkin_messages_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/RecordFeedback.msg" NAME_WE)
add_dependencies(pumpkin_messages_generate_messages_py _pumpkin_messages_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rafaelpaiva/workspace/pumpkin_ws/src/pumpkin_messages/msg/analog_array.msg" NAME_WE)
add_dependencies(pumpkin_messages_generate_messages_py _pumpkin_messages_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/PlaybackActionGoal.msg" NAME_WE)
add_dependencies(pumpkin_messages_generate_messages_py _pumpkin_messages_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/RecordActionGoal.msg" NAME_WE)
add_dependencies(pumpkin_messages_generate_messages_py _pumpkin_messages_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rafaelpaiva/workspace/pumpkin_ws/src/pumpkin_messages/srv/SSCMoveCommand.srv" NAME_WE)
add_dependencies(pumpkin_messages_generate_messages_py _pumpkin_messages_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rafaelpaiva/workspace/pumpkin_ws/src/pumpkin_messages/srv/Serials.srv" NAME_WE)
add_dependencies(pumpkin_messages_generate_messages_py _pumpkin_messages_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rafaelpaiva/workspace/pumpkin_ws/src/pumpkin_messages/msg/SSCMove.msg" NAME_WE)
add_dependencies(pumpkin_messages_generate_messages_py _pumpkin_messages_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rafaelpaiva/workspace/pumpkin_ws/src/pumpkin_messages/srv/Files.srv" NAME_WE)
add_dependencies(pumpkin_messages_generate_messages_py _pumpkin_messages_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/PlaybackFeedback.msg" NAME_WE)
add_dependencies(pumpkin_messages_generate_messages_py _pumpkin_messages_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/PlaybackAction.msg" NAME_WE)
add_dependencies(pumpkin_messages_generate_messages_py _pumpkin_messages_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rafaelpaiva/workspace/pumpkin_ws/src/pumpkin_messages/msg/SSCMoveList.msg" NAME_WE)
add_dependencies(pumpkin_messages_generate_messages_py _pumpkin_messages_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/RecordGoal.msg" NAME_WE)
add_dependencies(pumpkin_messages_generate_messages_py _pumpkin_messages_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/PlaybackResult.msg" NAME_WE)
add_dependencies(pumpkin_messages_generate_messages_py _pumpkin_messages_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/RecordResult.msg" NAME_WE)
add_dependencies(pumpkin_messages_generate_messages_py _pumpkin_messages_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(pumpkin_messages_genpy)
add_dependencies(pumpkin_messages_genpy pumpkin_messages_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS pumpkin_messages_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pumpkin_messages)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pumpkin_messages
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(pumpkin_messages_generate_messages_cpp std_msgs_generate_messages_cpp)
add_dependencies(pumpkin_messages_generate_messages_cpp actionlib_msgs_generate_messages_cpp)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pumpkin_messages)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pumpkin_messages
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(pumpkin_messages_generate_messages_lisp std_msgs_generate_messages_lisp)
add_dependencies(pumpkin_messages_generate_messages_lisp actionlib_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pumpkin_messages)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pumpkin_messages\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pumpkin_messages
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(pumpkin_messages_generate_messages_py std_msgs_generate_messages_py)
add_dependencies(pumpkin_messages_generate_messages_py actionlib_msgs_generate_messages_py)
