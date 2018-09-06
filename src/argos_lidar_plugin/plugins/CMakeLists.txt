### START - Adapted from argos3-examples/CMakeLists.txt

cmake_minimum_required(VERSION 2.8.12)
project(argos_lidar_plugin)

# Deactivate RPATH for MacOSX
set(CMAKE_MACOSX_RPATH 0)

# Set the path where CMake will find additional scripts
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR}/cmake)

# Find the ARGoS package, make sure to save the ARGoS prefix
find_package(PkgConfig)
pkg_check_modules(ARGOS REQUIRED argos3_simulator)
set(ARGOS_PREFIX ${ARGOS_PREFIX} CACHE INTERNAL "")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${ARGOS_PREFIX}/share/argos3/cmake)

# Check whether all the necessary libs have been installed to compile the
# code that depends on Qt and OpenGL
#include(ARGoSCheckQTOpenGL)

# Find the GSL package
find_package(GSL REQUIRED)

# Find Lua
find_package(Lua52 REQUIRED)

# Set ARGoS include dir
include_directories(${catkin_INCLUDE_DIRS} ${CMAKE_SOURCE_DIR} ${ARGOS_INCLUDE_DIRS} ${GSL_INCLUDE_DIR} ${LUA_INCLUDE_DIR})

# Set ARGoS link dir
link_directories(${ARGOS_LIBRARY_DIRS})

### END

add_library(argos_lidar_plugin SHARED 
  robots/generic/control_interface/ci_robot_lidar_sensor.h
  robots/generic/control_interface/ci_robot_lidar_sensor.cpp

  robots/generic/simulator/sensors/robot_lidar_default_sensor.h
  robots/generic/simulator/sensors/robot_lidar_default_sensor.cpp

  simulator/entities/lidar_sensor_equipped_entity.h 
  simulator/entities/lidar_sensor_equipped_entity.cpp
  )

target_link_libraries(
  argos_lidar_plugin
  argos3core_simulator
  /opt/ros/$ENV{ROS_DISTRO}/lib/libroscpp.so
  ${catkin_LIBRARIES}
)

install(
TARGETS argos_lidar_plugin
RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}/argos3_plugins
)