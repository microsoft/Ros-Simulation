add_library(argos_ros_bot MODULE argos_ros_bot.h argos_ros_bot.cpp)
target_link_libraries(argos_ros_bot
  argos_ground_truth_plugin
  argos_lidar_plugin
  argos_turtlebot3_wafflepi_plugin
  argos3core_simulator
  argos3plugin_simulator_genericrobot
  /opt/ros/$ENV{ROS_DISTRO}/lib/libroscpp.so
  ${catkin_LIBRARIES}
)

install(
TARGETS argos_ros_bot
RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}/argos3_plugins
)