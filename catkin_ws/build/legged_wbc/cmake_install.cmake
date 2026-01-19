# Install script for directory: /media/keaton/KEATON/11_Design/jet-hr2/catkin_ws/src/legged_wbc

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/media/keaton/KEATON/11_Design/jet-hr2/catkin_ws/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "RelWithDebInfo")
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
  
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
        file(MAKE_DIRECTORY "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
      endif()
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin")
        file(WRITE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin" "")
      endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/media/keaton/KEATON/11_Design/jet-hr2/catkin_ws/install/_setup_util.py")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/media/keaton/KEATON/11_Design/jet-hr2/catkin_ws/install" TYPE PROGRAM FILES "/media/keaton/KEATON/11_Design/jet-hr2/catkin_ws/build/legged_wbc/catkin_generated/installspace/_setup_util.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/media/keaton/KEATON/11_Design/jet-hr2/catkin_ws/install/env.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/media/keaton/KEATON/11_Design/jet-hr2/catkin_ws/install" TYPE PROGRAM FILES "/media/keaton/KEATON/11_Design/jet-hr2/catkin_ws/build/legged_wbc/catkin_generated/installspace/env.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/media/keaton/KEATON/11_Design/jet-hr2/catkin_ws/install/setup.bash;/media/keaton/KEATON/11_Design/jet-hr2/catkin_ws/install/local_setup.bash")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/media/keaton/KEATON/11_Design/jet-hr2/catkin_ws/install" TYPE FILE FILES
    "/media/keaton/KEATON/11_Design/jet-hr2/catkin_ws/build/legged_wbc/catkin_generated/installspace/setup.bash"
    "/media/keaton/KEATON/11_Design/jet-hr2/catkin_ws/build/legged_wbc/catkin_generated/installspace/local_setup.bash"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/media/keaton/KEATON/11_Design/jet-hr2/catkin_ws/install/setup.sh;/media/keaton/KEATON/11_Design/jet-hr2/catkin_ws/install/local_setup.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/media/keaton/KEATON/11_Design/jet-hr2/catkin_ws/install" TYPE FILE FILES
    "/media/keaton/KEATON/11_Design/jet-hr2/catkin_ws/build/legged_wbc/catkin_generated/installspace/setup.sh"
    "/media/keaton/KEATON/11_Design/jet-hr2/catkin_ws/build/legged_wbc/catkin_generated/installspace/local_setup.sh"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/media/keaton/KEATON/11_Design/jet-hr2/catkin_ws/install/setup.zsh;/media/keaton/KEATON/11_Design/jet-hr2/catkin_ws/install/local_setup.zsh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/media/keaton/KEATON/11_Design/jet-hr2/catkin_ws/install" TYPE FILE FILES
    "/media/keaton/KEATON/11_Design/jet-hr2/catkin_ws/build/legged_wbc/catkin_generated/installspace/setup.zsh"
    "/media/keaton/KEATON/11_Design/jet-hr2/catkin_ws/build/legged_wbc/catkin_generated/installspace/local_setup.zsh"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/media/keaton/KEATON/11_Design/jet-hr2/catkin_ws/install/setup.fish;/media/keaton/KEATON/11_Design/jet-hr2/catkin_ws/install/local_setup.fish")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/media/keaton/KEATON/11_Design/jet-hr2/catkin_ws/install" TYPE FILE FILES
    "/media/keaton/KEATON/11_Design/jet-hr2/catkin_ws/build/legged_wbc/catkin_generated/installspace/setup.fish"
    "/media/keaton/KEATON/11_Design/jet-hr2/catkin_ws/build/legged_wbc/catkin_generated/installspace/local_setup.fish"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/media/keaton/KEATON/11_Design/jet-hr2/catkin_ws/install/.rosinstall")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/media/keaton/KEATON/11_Design/jet-hr2/catkin_ws/install" TYPE FILE FILES "/media/keaton/KEATON/11_Design/jet-hr2/catkin_ws/build/legged_wbc/catkin_generated/installspace/.rosinstall")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/legged_wbc/msg" TYPE FILE FILES
    "/media/keaton/KEATON/11_Design/jet-hr2/catkin_ws/src/legged_wbc/msg/vector3f.msg"
    "/media/keaton/KEATON/11_Design/jet-hr2/catkin_ws/src/legged_wbc/msg/nvector3f.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/legged_wbc/cmake" TYPE FILE FILES "/media/keaton/KEATON/11_Design/jet-hr2/catkin_ws/build/legged_wbc/catkin_generated/installspace/legged_wbc-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/media/keaton/KEATON/11_Design/jet-hr2/catkin_ws/devel/.private/legged_wbc/include/legged_wbc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/media/keaton/KEATON/11_Design/jet-hr2/catkin_ws/devel/.private/legged_wbc/share/roseus/ros/legged_wbc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/media/keaton/KEATON/11_Design/jet-hr2/catkin_ws/devel/.private/legged_wbc/share/common-lisp/ros/legged_wbc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/media/keaton/KEATON/11_Design/jet-hr2/catkin_ws/devel/.private/legged_wbc/share/gennodejs/ros/legged_wbc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python3" -m compileall "/media/keaton/KEATON/11_Design/jet-hr2/catkin_ws/devel/.private/legged_wbc/lib/python3/dist-packages/legged_wbc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/media/keaton/KEATON/11_Design/jet-hr2/catkin_ws/devel/.private/legged_wbc/lib/python3/dist-packages/legged_wbc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/media/keaton/KEATON/11_Design/jet-hr2/catkin_ws/build/legged_wbc/catkin_generated/installspace/legged_wbc.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/legged_wbc/cmake" TYPE FILE FILES "/media/keaton/KEATON/11_Design/jet-hr2/catkin_ws/build/legged_wbc/catkin_generated/installspace/legged_wbc-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/legged_wbc/cmake" TYPE FILE FILES
    "/media/keaton/KEATON/11_Design/jet-hr2/catkin_ws/build/legged_wbc/catkin_generated/installspace/legged_wbcConfig.cmake"
    "/media/keaton/KEATON/11_Design/jet-hr2/catkin_ws/build/legged_wbc/catkin_generated/installspace/legged_wbcConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/legged_wbc" TYPE FILE FILES "/media/keaton/KEATON/11_Design/jet-hr2/catkin_ws/src/legged_wbc/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/liblegged_wbc.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/liblegged_wbc.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/liblegged_wbc.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/media/keaton/KEATON/11_Design/jet-hr2/catkin_ws/devel/.private/legged_wbc/lib/liblegged_wbc.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/liblegged_wbc.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/liblegged_wbc.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/liblegged_wbc.so"
         OLD_RPATH "/home/keaton/ocs2_ws/devel/lib:/usr/local/lib:/media/keaton/KEATON/11_Design/jet-hr2/catkin_ws/devel/.private/legged_interface/lib:/home/keaton/ocs2_ws/devel/.private/ocs2_legged_robot_ros/lib:/opt/ros/noetic/lib:/home/keaton/ocs2_ws/devel/.private/ocs2_legged_robot/lib:/home/keaton/ocs2_ws/devel/.private/ocs2_ddp/lib:/home/keaton/ocs2_ws/devel/.private/ocs2_sqp/lib:/home/keaton/ocs2_ws/devel/.private/ocs2_qp_solver/lib:/home/keaton/ocs2_ws/devel/.private/ocs2_ipm/lib:/home/keaton/ocs2_ws/devel/.private/hpipm_catkin/lib:/home/keaton/ocs2_ws/devel/.private/blasfeo_catkin/lib:/home/keaton/ocs2_ws/devel/.private/ocs2_centroidal_model/lib:/home/keaton/ocs2_ws/devel/.private/ocs2_self_collision/lib:/home/keaton/ocs2_ws/devel/.private/ocs2_pinocchio_interface/lib:/home/keaton/ocs2_ws/devel/.private/ocs2_robotic_tools/lib:/home/keaton/ocs2_ws/devel/.private/ocs2_ros_interfaces/lib:/home/keaton/ocs2_ws/devel/.private/ocs2_mpc/lib:/home/keaton/ocs2_ws/devel/.private/ocs2_oc/lib:/home/keaton/ocs2_ws/devel/.private/ocs2_core/lib:/media/keaton/KEATON/11_Design/jet-hr2/catkin_ws/devel/.private/qpoases_catkin/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/liblegged_wbc.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/legged_wbc" TYPE DIRECTORY FILES "/media/keaton/KEATON/11_Design/jet-hr2/catkin_ws/src/legged_wbc/include/legged_wbc/" FILES_MATCHING REGEX "/[^/]*\\.h$")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/media/keaton/KEATON/11_Design/jet-hr2/catkin_ws/build/legged_wbc/gtest/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/media/keaton/KEATON/11_Design/jet-hr2/catkin_ws/build/legged_wbc/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
