# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.16.3)

# We name the project and the target for the ExternalProject_Add() call
# to something that will highlight to the user what we are working on if
# something goes wrong and an error message is produced.

project(qpoasesdownload-populate NONE)

include(ExternalProject)
ExternalProject_Add(qpoasesdownload-populate
                     "UPDATE_DISCONNECTED" "False" "GIT_REPOSITORY" "https://github.com/coin-or/qpOASES" "GIT_TAG" "268b2f2659604df27c82aa6e32aeddb8c1d5cc7f" "UPDATE_COMMAND"
                    SOURCE_DIR          "/media/keaton/KEATON/11_Design/jet-hr2/catkin_ws/build/qpoases_catkin/download"
                    BINARY_DIR          "/media/keaton/KEATON/11_Design/jet-hr2/catkin_ws/build/qpoases_catkin/build"
                    CONFIGURE_COMMAND   ""
                    BUILD_COMMAND       ""
                    INSTALL_COMMAND     ""
                    TEST_COMMAND        ""
                    USES_TERMINAL_DOWNLOAD  YES
                    USES_TERMINAL_UPDATE    YES
)
