# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "id_data_msgs: 2 messages, 0 services")

set(MSG_I_FLAGS "-Iid_data_msgs:/home/robot/catkin_ws_base/src/id_data_msgs/msg;-Igeometry_msgs:/opt/ros/indigo/share/geometry_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/indigo/share/sensor_msgs/cmake/../msg;-Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(id_data_msgs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/robot/catkin_ws_base/src/id_data_msgs/msg/forceData.msg" NAME_WE)
add_custom_target(_id_data_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "id_data_msgs" "/home/robot/catkin_ws_base/src/id_data_msgs/msg/forceData.msg" ""
)

get_filename_component(_filename "/home/robot/catkin_ws_base/src/id_data_msgs/msg/ID_Data.msg" NAME_WE)
add_custom_target(_id_data_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "id_data_msgs" "/home/robot/catkin_ws_base/src/id_data_msgs/msg/ID_Data.msg" ""
)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(id_data_msgs
  "/home/robot/catkin_ws_base/src/id_data_msgs/msg/forceData.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/id_data_msgs
)
_generate_msg_cpp(id_data_msgs
  "/home/robot/catkin_ws_base/src/id_data_msgs/msg/ID_Data.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/id_data_msgs
)

### Generating Services

### Generating Module File
_generate_module_cpp(id_data_msgs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/id_data_msgs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(id_data_msgs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(id_data_msgs_generate_messages id_data_msgs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/robot/catkin_ws_base/src/id_data_msgs/msg/forceData.msg" NAME_WE)
add_dependencies(id_data_msgs_generate_messages_cpp _id_data_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robot/catkin_ws_base/src/id_data_msgs/msg/ID_Data.msg" NAME_WE)
add_dependencies(id_data_msgs_generate_messages_cpp _id_data_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(id_data_msgs_gencpp)
add_dependencies(id_data_msgs_gencpp id_data_msgs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS id_data_msgs_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(id_data_msgs
  "/home/robot/catkin_ws_base/src/id_data_msgs/msg/forceData.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/id_data_msgs
)
_generate_msg_lisp(id_data_msgs
  "/home/robot/catkin_ws_base/src/id_data_msgs/msg/ID_Data.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/id_data_msgs
)

### Generating Services

### Generating Module File
_generate_module_lisp(id_data_msgs
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/id_data_msgs
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(id_data_msgs_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(id_data_msgs_generate_messages id_data_msgs_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/robot/catkin_ws_base/src/id_data_msgs/msg/forceData.msg" NAME_WE)
add_dependencies(id_data_msgs_generate_messages_lisp _id_data_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robot/catkin_ws_base/src/id_data_msgs/msg/ID_Data.msg" NAME_WE)
add_dependencies(id_data_msgs_generate_messages_lisp _id_data_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(id_data_msgs_genlisp)
add_dependencies(id_data_msgs_genlisp id_data_msgs_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS id_data_msgs_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(id_data_msgs
  "/home/robot/catkin_ws_base/src/id_data_msgs/msg/forceData.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/id_data_msgs
)
_generate_msg_py(id_data_msgs
  "/home/robot/catkin_ws_base/src/id_data_msgs/msg/ID_Data.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/id_data_msgs
)

### Generating Services

### Generating Module File
_generate_module_py(id_data_msgs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/id_data_msgs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(id_data_msgs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(id_data_msgs_generate_messages id_data_msgs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/robot/catkin_ws_base/src/id_data_msgs/msg/forceData.msg" NAME_WE)
add_dependencies(id_data_msgs_generate_messages_py _id_data_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robot/catkin_ws_base/src/id_data_msgs/msg/ID_Data.msg" NAME_WE)
add_dependencies(id_data_msgs_generate_messages_py _id_data_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(id_data_msgs_genpy)
add_dependencies(id_data_msgs_genpy id_data_msgs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS id_data_msgs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/id_data_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/id_data_msgs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(id_data_msgs_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(id_data_msgs_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(id_data_msgs_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/id_data_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/id_data_msgs
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(id_data_msgs_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(id_data_msgs_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(id_data_msgs_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/id_data_msgs)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/id_data_msgs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/id_data_msgs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(id_data_msgs_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(id_data_msgs_generate_messages_py sensor_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(id_data_msgs_generate_messages_py std_msgs_generate_messages_py)
endif()
