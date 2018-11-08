Introduction
------------

The purpose of this package is to connect ROS with the ARGoS (henceforth argos)
robot simulator.  

    http://www.argos-sim.info

The basic concept is the use of an argos plugin (in the 'plugin' directory).
When argos is executed it looks at ARGOS_PLUGIN_PATH which you should have
configured to find this plugin.  The plugin represents a single robot which
subscribes and publishes to ROS topics.  If the argos world has multiple robots,
they will each have a separate instance of the C++ class in the 'plugin' dir
which connects to ROS.

Setup
-----

It is assumed that argos has been installed.  

The contents of this directory should be placed within your ROS catkin workspace src directory (e.g. ~/catkin_ws/src).

The directory 'ros_lib_links' has a symbolic link to libroscpp.so.  This may
need to be updated for your system (most likely).

The following environment variables should be set, replacing 'catkin_ws' with a
suitable value if necessary:

    export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib/argos3
    export ARGOS_PLUGIN_PATH=$HOME/catkin_ws/src/argos_bridge/ros_lib_links
    export ARGOS_PLUGIN_PATH=$ARGOS_PLUGIN_PATH:$HOME/catkin_ws/devel/lib

Of course, you had better check that all of these paths make sense.

Use
---
Once the package has been built (with catkin_make --- see "Issues" below) do
the following, or some variation:

    cd ~/catkin_ws/src/argos_bridge
    ./launch/launch_demo.sh

This executes a script which generates a launch file which is passed to
roslaunch for execution.  Each robot will be running a different instance of
'demo_controller.py' from the 'scripts' directory.
    
    argos3 -c argos_worlds/construct.argos
    
Hit the play button on the simulator.  

Issues
------
- There is some issue in the order of building the messages in 'msg' and the
  argos plugin in 'plugin'.  To resolve, comment out the following line in
  CMakeLists.txt:

    add_subdirectory(plugin)

  Go ahead and do 'catkin_make' then uncomment the line and do another
  'catkin_make'.  This should only occur once.

- If there are issues related to ros::console::initialized, check your ROS version
and if the ROS_DISTRO env is set.   

Author
------
Andrew Vardy (av@mun.ca)
Department of Computer Science
Department of Electrical and Computer Engineering
Memorial University
St. John's, Canada
