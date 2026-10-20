# AltitudeLiftDrag: gz-sim's LiftDrag with the air density taken from height
# (see AltitudeLiftDrag.cc).  Builds one system plugin, libAltitudeLiftDrag.so,
# outside the Gazebo source tree.
#
# Build and install, from this directory, in the shell Gazebo runs in:
#
#   cmake -S . -B ~/build/AltitudeLiftDrag
#   cmake --build ~/build/AltitudeLiftDrag
#   cmake --install ~/build/AltitudeLiftDrag
#
# The build folder sits outside the project so the project holds sources only.
# The install copies the library to ~/.gz/sim/plugins, which gz-sim searches
# for system plugins after GZ_SIM_SYSTEM_PLUGIN_PATH.  Set
# ALTITUDE_LIFT_DRAG_INSTALL_DIR to put it somewhere else.
#
# The Gazebo release is chosen the way ardupilot_gazebo chooses it: from
# GZ_VERSION (harmonic when unset, or garden, ionic, jetty).  Build against the
# same release ardupilot_gazebo was built against.

cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)
project(AltitudeLiftDrag LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if("$ENV{GZ_VERSION}" STREQUAL "harmonic" OR NOT DEFINED ENV{GZ_VERSION})
  set(GZ_SIM_PACKAGE gz-sim8)
elseif("$ENV{GZ_VERSION}" STREQUAL "ionic")
  set(GZ_SIM_PACKAGE gz-sim9)
elseif("$ENV{GZ_VERSION}" STREQUAL "garden")
  set(GZ_SIM_PACKAGE gz-sim7)
elseif("$ENV{GZ_VERSION}" STREQUAL "jetty")
  set(GZ_SIM_PACKAGE gz-sim)
else()
  message(FATAL_ERROR "Unsupported GZ_VERSION: $ENV{GZ_VERSION}")
endif()

find_package(${GZ_SIM_PACKAGE} REQUIRED)
message(STATUS "Compiling AltitudeLiftDrag against ${GZ_SIM_PACKAGE}")

add_library(AltitudeLiftDrag SHARED AltitudeLiftDrag.cc)
target_link_libraries(AltitudeLiftDrag PRIVATE
  ${GZ_SIM_PACKAGE}::${GZ_SIM_PACKAGE}
)

set(ALTITUDE_LIFT_DRAG_INSTALL_DIR "$ENV{HOME}/.gz/sim/plugins"
    CACHE PATH "Where cmake --install puts libAltitudeLiftDrag.so")
install(TARGETS AltitudeLiftDrag
        LIBRARY DESTINATION "${ALTITUDE_LIFT_DRAG_INSTALL_DIR}")
