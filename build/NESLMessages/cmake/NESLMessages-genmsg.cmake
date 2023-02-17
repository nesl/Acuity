# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "NESLMessages: 3 messages, 0 services")

set(MSG_I_FLAGS "-INESLMessages:/home/nesl/Desktop/Acuity/src/NESLMessages/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(NESLMessages_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/nesl/Desktop/Acuity/src/NESLMessages/msg/NeslCoord.msg" NAME_WE)
add_custom_target(_NESLMessages_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "NESLMessages" "/home/nesl/Desktop/Acuity/src/NESLMessages/msg/NeslCoord.msg" ""
)

get_filename_component(_filename "/home/nesl/Desktop/Acuity/src/NESLMessages/msg/Person.msg" NAME_WE)
add_custom_target(_NESLMessages_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "NESLMessages" "/home/nesl/Desktop/Acuity/src/NESLMessages/msg/Person.msg" "NESLMessages/NeslCoord"
)

get_filename_component(_filename "/home/nesl/Desktop/Acuity/src/NESLMessages/msg/PersonArr.msg" NAME_WE)
add_custom_target(_NESLMessages_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "NESLMessages" "/home/nesl/Desktop/Acuity/src/NESLMessages/msg/PersonArr.msg" "NESLMessages/NeslCoord:std_msgs/Header:NESLMessages/Person"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(NESLMessages
  "/home/nesl/Desktop/Acuity/src/NESLMessages/msg/NeslCoord.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/NESLMessages
)
_generate_msg_cpp(NESLMessages
  "/home/nesl/Desktop/Acuity/src/NESLMessages/msg/Person.msg"
  "${MSG_I_FLAGS}"
  "/home/nesl/Desktop/Acuity/src/NESLMessages/msg/NeslCoord.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/NESLMessages
)
_generate_msg_cpp(NESLMessages
  "/home/nesl/Desktop/Acuity/src/NESLMessages/msg/PersonArr.msg"
  "${MSG_I_FLAGS}"
  "/home/nesl/Desktop/Acuity/src/NESLMessages/msg/NeslCoord.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/nesl/Desktop/Acuity/src/NESLMessages/msg/Person.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/NESLMessages
)

### Generating Services

### Generating Module File
_generate_module_cpp(NESLMessages
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/NESLMessages
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(NESLMessages_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(NESLMessages_generate_messages NESLMessages_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/nesl/Desktop/Acuity/src/NESLMessages/msg/NeslCoord.msg" NAME_WE)
add_dependencies(NESLMessages_generate_messages_cpp _NESLMessages_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nesl/Desktop/Acuity/src/NESLMessages/msg/Person.msg" NAME_WE)
add_dependencies(NESLMessages_generate_messages_cpp _NESLMessages_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nesl/Desktop/Acuity/src/NESLMessages/msg/PersonArr.msg" NAME_WE)
add_dependencies(NESLMessages_generate_messages_cpp _NESLMessages_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(NESLMessages_gencpp)
add_dependencies(NESLMessages_gencpp NESLMessages_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS NESLMessages_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(NESLMessages
  "/home/nesl/Desktop/Acuity/src/NESLMessages/msg/NeslCoord.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/NESLMessages
)
_generate_msg_eus(NESLMessages
  "/home/nesl/Desktop/Acuity/src/NESLMessages/msg/Person.msg"
  "${MSG_I_FLAGS}"
  "/home/nesl/Desktop/Acuity/src/NESLMessages/msg/NeslCoord.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/NESLMessages
)
_generate_msg_eus(NESLMessages
  "/home/nesl/Desktop/Acuity/src/NESLMessages/msg/PersonArr.msg"
  "${MSG_I_FLAGS}"
  "/home/nesl/Desktop/Acuity/src/NESLMessages/msg/NeslCoord.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/nesl/Desktop/Acuity/src/NESLMessages/msg/Person.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/NESLMessages
)

### Generating Services

### Generating Module File
_generate_module_eus(NESLMessages
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/NESLMessages
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(NESLMessages_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(NESLMessages_generate_messages NESLMessages_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/nesl/Desktop/Acuity/src/NESLMessages/msg/NeslCoord.msg" NAME_WE)
add_dependencies(NESLMessages_generate_messages_eus _NESLMessages_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nesl/Desktop/Acuity/src/NESLMessages/msg/Person.msg" NAME_WE)
add_dependencies(NESLMessages_generate_messages_eus _NESLMessages_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nesl/Desktop/Acuity/src/NESLMessages/msg/PersonArr.msg" NAME_WE)
add_dependencies(NESLMessages_generate_messages_eus _NESLMessages_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(NESLMessages_geneus)
add_dependencies(NESLMessages_geneus NESLMessages_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS NESLMessages_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(NESLMessages
  "/home/nesl/Desktop/Acuity/src/NESLMessages/msg/NeslCoord.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/NESLMessages
)
_generate_msg_lisp(NESLMessages
  "/home/nesl/Desktop/Acuity/src/NESLMessages/msg/Person.msg"
  "${MSG_I_FLAGS}"
  "/home/nesl/Desktop/Acuity/src/NESLMessages/msg/NeslCoord.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/NESLMessages
)
_generate_msg_lisp(NESLMessages
  "/home/nesl/Desktop/Acuity/src/NESLMessages/msg/PersonArr.msg"
  "${MSG_I_FLAGS}"
  "/home/nesl/Desktop/Acuity/src/NESLMessages/msg/NeslCoord.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/nesl/Desktop/Acuity/src/NESLMessages/msg/Person.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/NESLMessages
)

### Generating Services

### Generating Module File
_generate_module_lisp(NESLMessages
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/NESLMessages
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(NESLMessages_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(NESLMessages_generate_messages NESLMessages_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/nesl/Desktop/Acuity/src/NESLMessages/msg/NeslCoord.msg" NAME_WE)
add_dependencies(NESLMessages_generate_messages_lisp _NESLMessages_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nesl/Desktop/Acuity/src/NESLMessages/msg/Person.msg" NAME_WE)
add_dependencies(NESLMessages_generate_messages_lisp _NESLMessages_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nesl/Desktop/Acuity/src/NESLMessages/msg/PersonArr.msg" NAME_WE)
add_dependencies(NESLMessages_generate_messages_lisp _NESLMessages_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(NESLMessages_genlisp)
add_dependencies(NESLMessages_genlisp NESLMessages_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS NESLMessages_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(NESLMessages
  "/home/nesl/Desktop/Acuity/src/NESLMessages/msg/NeslCoord.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/NESLMessages
)
_generate_msg_nodejs(NESLMessages
  "/home/nesl/Desktop/Acuity/src/NESLMessages/msg/Person.msg"
  "${MSG_I_FLAGS}"
  "/home/nesl/Desktop/Acuity/src/NESLMessages/msg/NeslCoord.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/NESLMessages
)
_generate_msg_nodejs(NESLMessages
  "/home/nesl/Desktop/Acuity/src/NESLMessages/msg/PersonArr.msg"
  "${MSG_I_FLAGS}"
  "/home/nesl/Desktop/Acuity/src/NESLMessages/msg/NeslCoord.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/nesl/Desktop/Acuity/src/NESLMessages/msg/Person.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/NESLMessages
)

### Generating Services

### Generating Module File
_generate_module_nodejs(NESLMessages
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/NESLMessages
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(NESLMessages_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(NESLMessages_generate_messages NESLMessages_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/nesl/Desktop/Acuity/src/NESLMessages/msg/NeslCoord.msg" NAME_WE)
add_dependencies(NESLMessages_generate_messages_nodejs _NESLMessages_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nesl/Desktop/Acuity/src/NESLMessages/msg/Person.msg" NAME_WE)
add_dependencies(NESLMessages_generate_messages_nodejs _NESLMessages_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nesl/Desktop/Acuity/src/NESLMessages/msg/PersonArr.msg" NAME_WE)
add_dependencies(NESLMessages_generate_messages_nodejs _NESLMessages_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(NESLMessages_gennodejs)
add_dependencies(NESLMessages_gennodejs NESLMessages_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS NESLMessages_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(NESLMessages
  "/home/nesl/Desktop/Acuity/src/NESLMessages/msg/NeslCoord.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/NESLMessages
)
_generate_msg_py(NESLMessages
  "/home/nesl/Desktop/Acuity/src/NESLMessages/msg/Person.msg"
  "${MSG_I_FLAGS}"
  "/home/nesl/Desktop/Acuity/src/NESLMessages/msg/NeslCoord.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/NESLMessages
)
_generate_msg_py(NESLMessages
  "/home/nesl/Desktop/Acuity/src/NESLMessages/msg/PersonArr.msg"
  "${MSG_I_FLAGS}"
  "/home/nesl/Desktop/Acuity/src/NESLMessages/msg/NeslCoord.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/nesl/Desktop/Acuity/src/NESLMessages/msg/Person.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/NESLMessages
)

### Generating Services

### Generating Module File
_generate_module_py(NESLMessages
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/NESLMessages
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(NESLMessages_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(NESLMessages_generate_messages NESLMessages_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/nesl/Desktop/Acuity/src/NESLMessages/msg/NeslCoord.msg" NAME_WE)
add_dependencies(NESLMessages_generate_messages_py _NESLMessages_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nesl/Desktop/Acuity/src/NESLMessages/msg/Person.msg" NAME_WE)
add_dependencies(NESLMessages_generate_messages_py _NESLMessages_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nesl/Desktop/Acuity/src/NESLMessages/msg/PersonArr.msg" NAME_WE)
add_dependencies(NESLMessages_generate_messages_py _NESLMessages_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(NESLMessages_genpy)
add_dependencies(NESLMessages_genpy NESLMessages_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS NESLMessages_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/NESLMessages)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/NESLMessages
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(NESLMessages_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/NESLMessages)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/NESLMessages
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(NESLMessages_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/NESLMessages)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/NESLMessages
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(NESLMessages_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/NESLMessages)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/NESLMessages
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(NESLMessages_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/NESLMessages)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/NESLMessages\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/NESLMessages
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(NESLMessages_generate_messages_py std_msgs_generate_messages_py)
endif()
