# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "legged_wbc: 2 messages, 0 services")

set(MSG_I_FLAGS "-Ilegged_wbc:/media/keaton/KEATON/11_Design/jet-hr2/catkin_ws/src/legged_wbc/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(legged_wbc_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/media/keaton/KEATON/11_Design/jet-hr2/catkin_ws/src/legged_wbc/msg/vector3f.msg" NAME_WE)
add_custom_target(_legged_wbc_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "legged_wbc" "/media/keaton/KEATON/11_Design/jet-hr2/catkin_ws/src/legged_wbc/msg/vector3f.msg" ""
)

get_filename_component(_filename "/media/keaton/KEATON/11_Design/jet-hr2/catkin_ws/src/legged_wbc/msg/nvector3f.msg" NAME_WE)
add_custom_target(_legged_wbc_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "legged_wbc" "/media/keaton/KEATON/11_Design/jet-hr2/catkin_ws/src/legged_wbc/msg/nvector3f.msg" "legged_wbc/vector3f"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(legged_wbc
  "/media/keaton/KEATON/11_Design/jet-hr2/catkin_ws/src/legged_wbc/msg/vector3f.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/legged_wbc
)
_generate_msg_cpp(legged_wbc
  "/media/keaton/KEATON/11_Design/jet-hr2/catkin_ws/src/legged_wbc/msg/nvector3f.msg"
  "${MSG_I_FLAGS}"
  "/media/keaton/KEATON/11_Design/jet-hr2/catkin_ws/src/legged_wbc/msg/vector3f.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/legged_wbc
)

### Generating Services

### Generating Module File
_generate_module_cpp(legged_wbc
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/legged_wbc
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(legged_wbc_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(legged_wbc_generate_messages legged_wbc_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/media/keaton/KEATON/11_Design/jet-hr2/catkin_ws/src/legged_wbc/msg/vector3f.msg" NAME_WE)
add_dependencies(legged_wbc_generate_messages_cpp _legged_wbc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/keaton/KEATON/11_Design/jet-hr2/catkin_ws/src/legged_wbc/msg/nvector3f.msg" NAME_WE)
add_dependencies(legged_wbc_generate_messages_cpp _legged_wbc_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(legged_wbc_gencpp)
add_dependencies(legged_wbc_gencpp legged_wbc_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS legged_wbc_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(legged_wbc
  "/media/keaton/KEATON/11_Design/jet-hr2/catkin_ws/src/legged_wbc/msg/vector3f.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/legged_wbc
)
_generate_msg_eus(legged_wbc
  "/media/keaton/KEATON/11_Design/jet-hr2/catkin_ws/src/legged_wbc/msg/nvector3f.msg"
  "${MSG_I_FLAGS}"
  "/media/keaton/KEATON/11_Design/jet-hr2/catkin_ws/src/legged_wbc/msg/vector3f.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/legged_wbc
)

### Generating Services

### Generating Module File
_generate_module_eus(legged_wbc
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/legged_wbc
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(legged_wbc_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(legged_wbc_generate_messages legged_wbc_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/media/keaton/KEATON/11_Design/jet-hr2/catkin_ws/src/legged_wbc/msg/vector3f.msg" NAME_WE)
add_dependencies(legged_wbc_generate_messages_eus _legged_wbc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/keaton/KEATON/11_Design/jet-hr2/catkin_ws/src/legged_wbc/msg/nvector3f.msg" NAME_WE)
add_dependencies(legged_wbc_generate_messages_eus _legged_wbc_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(legged_wbc_geneus)
add_dependencies(legged_wbc_geneus legged_wbc_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS legged_wbc_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(legged_wbc
  "/media/keaton/KEATON/11_Design/jet-hr2/catkin_ws/src/legged_wbc/msg/vector3f.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/legged_wbc
)
_generate_msg_lisp(legged_wbc
  "/media/keaton/KEATON/11_Design/jet-hr2/catkin_ws/src/legged_wbc/msg/nvector3f.msg"
  "${MSG_I_FLAGS}"
  "/media/keaton/KEATON/11_Design/jet-hr2/catkin_ws/src/legged_wbc/msg/vector3f.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/legged_wbc
)

### Generating Services

### Generating Module File
_generate_module_lisp(legged_wbc
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/legged_wbc
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(legged_wbc_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(legged_wbc_generate_messages legged_wbc_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/media/keaton/KEATON/11_Design/jet-hr2/catkin_ws/src/legged_wbc/msg/vector3f.msg" NAME_WE)
add_dependencies(legged_wbc_generate_messages_lisp _legged_wbc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/keaton/KEATON/11_Design/jet-hr2/catkin_ws/src/legged_wbc/msg/nvector3f.msg" NAME_WE)
add_dependencies(legged_wbc_generate_messages_lisp _legged_wbc_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(legged_wbc_genlisp)
add_dependencies(legged_wbc_genlisp legged_wbc_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS legged_wbc_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(legged_wbc
  "/media/keaton/KEATON/11_Design/jet-hr2/catkin_ws/src/legged_wbc/msg/vector3f.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/legged_wbc
)
_generate_msg_nodejs(legged_wbc
  "/media/keaton/KEATON/11_Design/jet-hr2/catkin_ws/src/legged_wbc/msg/nvector3f.msg"
  "${MSG_I_FLAGS}"
  "/media/keaton/KEATON/11_Design/jet-hr2/catkin_ws/src/legged_wbc/msg/vector3f.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/legged_wbc
)

### Generating Services

### Generating Module File
_generate_module_nodejs(legged_wbc
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/legged_wbc
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(legged_wbc_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(legged_wbc_generate_messages legged_wbc_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/media/keaton/KEATON/11_Design/jet-hr2/catkin_ws/src/legged_wbc/msg/vector3f.msg" NAME_WE)
add_dependencies(legged_wbc_generate_messages_nodejs _legged_wbc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/keaton/KEATON/11_Design/jet-hr2/catkin_ws/src/legged_wbc/msg/nvector3f.msg" NAME_WE)
add_dependencies(legged_wbc_generate_messages_nodejs _legged_wbc_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(legged_wbc_gennodejs)
add_dependencies(legged_wbc_gennodejs legged_wbc_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS legged_wbc_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(legged_wbc
  "/media/keaton/KEATON/11_Design/jet-hr2/catkin_ws/src/legged_wbc/msg/vector3f.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/legged_wbc
)
_generate_msg_py(legged_wbc
  "/media/keaton/KEATON/11_Design/jet-hr2/catkin_ws/src/legged_wbc/msg/nvector3f.msg"
  "${MSG_I_FLAGS}"
  "/media/keaton/KEATON/11_Design/jet-hr2/catkin_ws/src/legged_wbc/msg/vector3f.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/legged_wbc
)

### Generating Services

### Generating Module File
_generate_module_py(legged_wbc
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/legged_wbc
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(legged_wbc_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(legged_wbc_generate_messages legged_wbc_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/media/keaton/KEATON/11_Design/jet-hr2/catkin_ws/src/legged_wbc/msg/vector3f.msg" NAME_WE)
add_dependencies(legged_wbc_generate_messages_py _legged_wbc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/keaton/KEATON/11_Design/jet-hr2/catkin_ws/src/legged_wbc/msg/nvector3f.msg" NAME_WE)
add_dependencies(legged_wbc_generate_messages_py _legged_wbc_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(legged_wbc_genpy)
add_dependencies(legged_wbc_genpy legged_wbc_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS legged_wbc_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/legged_wbc)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/legged_wbc
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(legged_wbc_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/legged_wbc)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/legged_wbc
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(legged_wbc_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/legged_wbc)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/legged_wbc
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(legged_wbc_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/legged_wbc)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/legged_wbc
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(legged_wbc_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/legged_wbc)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/legged_wbc\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/legged_wbc
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(legged_wbc_generate_messages_py std_msgs_generate_messages_py)
endif()
