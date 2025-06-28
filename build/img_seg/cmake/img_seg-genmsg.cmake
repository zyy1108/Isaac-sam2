# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "img_seg: 0 messages, 1 services")

set(MSG_I_FLAGS "-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(img_seg_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/gpu007-905/isaac_vision/src/img_seg/srv/Segment.srv" NAME_WE)
add_custom_target(_img_seg_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "img_seg" "/home/gpu007-905/isaac_vision/src/img_seg/srv/Segment.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(img_seg
  "/home/gpu007-905/isaac_vision/src/img_seg/srv/Segment.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/img_seg
)

### Generating Module File
_generate_module_cpp(img_seg
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/img_seg
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(img_seg_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(img_seg_generate_messages img_seg_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/gpu007-905/isaac_vision/src/img_seg/srv/Segment.srv" NAME_WE)
add_dependencies(img_seg_generate_messages_cpp _img_seg_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(img_seg_gencpp)
add_dependencies(img_seg_gencpp img_seg_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS img_seg_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages

### Generating Services
_generate_srv_eus(img_seg
  "/home/gpu007-905/isaac_vision/src/img_seg/srv/Segment.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/img_seg
)

### Generating Module File
_generate_module_eus(img_seg
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/img_seg
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(img_seg_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(img_seg_generate_messages img_seg_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/gpu007-905/isaac_vision/src/img_seg/srv/Segment.srv" NAME_WE)
add_dependencies(img_seg_generate_messages_eus _img_seg_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(img_seg_geneus)
add_dependencies(img_seg_geneus img_seg_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS img_seg_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(img_seg
  "/home/gpu007-905/isaac_vision/src/img_seg/srv/Segment.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/img_seg
)

### Generating Module File
_generate_module_lisp(img_seg
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/img_seg
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(img_seg_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(img_seg_generate_messages img_seg_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/gpu007-905/isaac_vision/src/img_seg/srv/Segment.srv" NAME_WE)
add_dependencies(img_seg_generate_messages_lisp _img_seg_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(img_seg_genlisp)
add_dependencies(img_seg_genlisp img_seg_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS img_seg_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages

### Generating Services
_generate_srv_nodejs(img_seg
  "/home/gpu007-905/isaac_vision/src/img_seg/srv/Segment.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/img_seg
)

### Generating Module File
_generate_module_nodejs(img_seg
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/img_seg
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(img_seg_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(img_seg_generate_messages img_seg_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/gpu007-905/isaac_vision/src/img_seg/srv/Segment.srv" NAME_WE)
add_dependencies(img_seg_generate_messages_nodejs _img_seg_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(img_seg_gennodejs)
add_dependencies(img_seg_gennodejs img_seg_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS img_seg_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(img_seg
  "/home/gpu007-905/isaac_vision/src/img_seg/srv/Segment.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/img_seg
)

### Generating Module File
_generate_module_py(img_seg
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/img_seg
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(img_seg_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(img_seg_generate_messages img_seg_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/gpu007-905/isaac_vision/src/img_seg/srv/Segment.srv" NAME_WE)
add_dependencies(img_seg_generate_messages_py _img_seg_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(img_seg_genpy)
add_dependencies(img_seg_genpy img_seg_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS img_seg_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/img_seg)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/img_seg
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(img_seg_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/img_seg)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/img_seg
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(img_seg_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/img_seg)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/img_seg
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(img_seg_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/img_seg)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/img_seg
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(img_seg_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/img_seg)
  install(CODE "execute_process(COMMAND \"/home/gpu007-905/miniconda3/envs/sam2_env/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/img_seg\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/img_seg
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(img_seg_generate_messages_py std_msgs_generate_messages_py)
endif()
