add_library(argos_ros_clock MODULE argos_ros_clock.h argos_ros_clock.cpp)
target_link_libraries(argos_ros_clock
  argos3core_simulator
  /opt/ros/$ENV{ROS_DISTRO}/lib/libroscpp.so)

install(
TARGETS argos_ros_clock
RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}/argos3_plugins
)