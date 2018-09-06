add_library(argos_turtlebot3_wafflepi_plugin SHARED 
  turtlebot/simulator/turtlebot_measures.h
  turtlebot/simulator/turtlebot_measures.cpp
  turtlebot/simulator/entities/turtlebot_entity.h
  turtlebot/simulator/entities/turtlebot_entity.cpp
  turtlebot/simulator/physics_engines/dynamics2d/dynamics2d_turtlebot_model.h
  turtlebot/simulator/physics_engines/dynamics2d/dynamics2d_turtlebot_model.cpp
  turtlebot/simulator/visualizations/qt-opengl/qtopengl_turtlebot.h
  turtlebot/simulator/visualizations/qt-opengl/qtopengl_turtlebot.cpp
  )

target_link_libraries(
  argos_turtlebot3_wafflepi_plugin
  argos_ground_truth_plugin
  argos_lidar_plugin
  argos3core_simulator
  argos3plugin_simulator_entities
  argos3plugin_simulator_dynamics2d
  argos3plugin_simulator_qtopengl
  /opt/ros/$ENV{ROS_DISTRO}/lib/libroscpp.so
  ${catkin_LIBRARIES}
)

install(
TARGETS argos_turtlebot3_wafflepi_plugin
RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}/argos3_plugins
)