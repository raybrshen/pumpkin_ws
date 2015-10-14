# Install script for directory: /home/rafaelpaiva/workspace/pumpkin_ws/src/pumpkin_messages

# Set the install prefix
IF(NOT DEFINED CMAKE_INSTALL_PREFIX)
  SET(CMAKE_INSTALL_PREFIX "/usr/local")
ENDIF(NOT DEFINED CMAKE_INSTALL_PREFIX)
STRING(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
IF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  IF(BUILD_TYPE)
    STRING(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  ELSE(BUILD_TYPE)
    SET(CMAKE_INSTALL_CONFIG_NAME "")
  ENDIF(BUILD_TYPE)
  MESSAGE(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
ENDIF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)

# Set the component getting installed.
IF(NOT CMAKE_INSTALL_COMPONENT)
  IF(COMPONENT)
    MESSAGE(STATUS "Install component: \"${COMPONENT}\"")
    SET(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  ELSE(COMPONENT)
    SET(CMAKE_INSTALL_COMPONENT)
  ENDIF(COMPONENT)
ENDIF(NOT CMAKE_INSTALL_COMPONENT)

# Install shared libraries without execute permission?
IF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  SET(CMAKE_INSTALL_SO_NO_EXE "1")
ENDIF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/pumpkin_messages/msg" TYPE FILE FILES
    "/home/rafaelpaiva/workspace/pumpkin_ws/src/pumpkin_messages/msg/analog_array.msg"
    "/home/rafaelpaiva/workspace/pumpkin_ws/src/pumpkin_messages/msg/SSCMove.msg"
    "/home/rafaelpaiva/workspace/pumpkin_ws/src/pumpkin_messages/msg/SSCMoveList.msg"
    "/home/rafaelpaiva/workspace/pumpkin_ws/src/pumpkin_messages/msg/FileList.msg"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/pumpkin_messages/srv" TYPE FILE FILES
    "/home/rafaelpaiva/workspace/pumpkin_ws/src/pumpkin_messages/srv/Files.srv"
    "/home/rafaelpaiva/workspace/pumpkin_ws/src/pumpkin_messages/srv/Serials.srv"
    "/home/rafaelpaiva/workspace/pumpkin_ws/src/pumpkin_messages/srv/SSCMoveCommand.srv"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/pumpkin_messages/action" TYPE FILE FILES
    "/home/rafaelpaiva/workspace/pumpkin_ws/src/pumpkin_messages/action/Record.action"
    "/home/rafaelpaiva/workspace/pumpkin_ws/src/pumpkin_messages/action/Playback.action"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/pumpkin_messages/msg" TYPE FILE FILES
    "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/RecordAction.msg"
    "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/RecordActionGoal.msg"
    "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/RecordActionResult.msg"
    "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/RecordActionFeedback.msg"
    "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/RecordGoal.msg"
    "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/RecordResult.msg"
    "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/RecordFeedback.msg"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/pumpkin_messages/msg" TYPE FILE FILES
    "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/PlaybackAction.msg"
    "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/PlaybackActionGoal.msg"
    "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/PlaybackActionResult.msg"
    "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/PlaybackActionFeedback.msg"
    "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/PlaybackGoal.msg"
    "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/PlaybackResult.msg"
    "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/pumpkin_messages/msg/PlaybackFeedback.msg"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/pumpkin_messages/cmake" TYPE FILE FILES "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/pumpkin_messages/catkin_generated/installspace/pumpkin_messages-msg-paths.cmake")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/include/pumpkin_messages")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/share/common-lisp/ros/pumpkin_messages")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  execute_process(COMMAND "/usr/bin/python" -m compileall "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/lib/python2.7/dist-packages/pumpkin_messages")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/devel/lib/python2.7/dist-packages/pumpkin_messages")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/pumpkin_messages/catkin_generated/installspace/pumpkin_messages.pc")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/pumpkin_messages/cmake" TYPE FILE FILES "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/pumpkin_messages/catkin_generated/installspace/pumpkin_messages-msg-extras.cmake")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/pumpkin_messages/cmake" TYPE FILE FILES
    "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/pumpkin_messages/catkin_generated/installspace/pumpkin_messagesConfig.cmake"
    "/home/rafaelpaiva/workspace/pumpkin_ws/build-src-Desktop-Default/pumpkin_messages/catkin_generated/installspace/pumpkin_messagesConfig-version.cmake"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/pumpkin_messages" TYPE FILE FILES "/home/rafaelpaiva/workspace/pumpkin_ws/src/pumpkin_messages/package.xml")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

