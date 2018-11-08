#!/bin/bash

# The purpose of this script is to create a launch file for a multi-robot
# system by replicating a "group" tag "n" times, then executing the resulting
# launch file.

# The number of robots.  This should match the 'quantity' value in the argos world file (e.g. argos_worlds/demo.argos).
n=${1:-3}

LAUNCH_FILE=/tmp/argos_bridge.launch

echo "<launch>" > $LAUNCH_FILE
echo "<rosparam param=\"use_sim_time\">true</rosparam>" >> $LAUNCH_FILE
for ((i=0; i<n; i++)); do
    namespace="robot$i"
    echo -e "\t<group ns=\"$namespace\">"
    echo -e "<param name=\"tf_prefix\" value=\"/"$namespace"\"/>"

    echo -e "\t\t<!--node pkg=\"simulated_robot\" \
        type=\"robot_controller.py\" name=\"robot_controller\" output=\"screen\" /-->"
    
    echo -e "\t\t<include file=\"\$(find simulated_robot)/planning/planning.launch\" >"
    echo -e "\t\t\t<arg name=\"ns\" value=\""$namespace"\"/>"
    echo -e "\t\t</include>"
    echo -e "\t</group>"
done >> $LAUNCH_FILE
echo -e "</launch>" >> $LAUNCH_FILE

roslaunch $LAUNCH_FILE
