#!/bin/bash

# Copyright (c) Microsoft Corporation. All rights reserved.
# Licensed under the MIT License.

# The number of robots.  This should match the 'quantity' value in the argos world file (e.g. argos_worlds/demo.argos).
N=${1:-3}
TARGET_PREFIX=${2:-robot-nimbro-}
ROBOT_PREFIX=${3:-hackbot}
PROTOCOL=${4:-tcp}

LAUNCH_DIR=$(rospack find simulator_nimbro)/launch 
LAUNCH_FILE=$LAUNCH_DIR/senders.launch

echo "<launch>" > "$LAUNCH_FILE"
for ((i=0; i<N; i++)); do
    target=$TARGET_PREFIX$i
    target_port=17000
    sender_name=${PROTOCOL}_sender$i
    ns=$ROBOT_PREFIX$i
    protocol=$PROTOCOL
    echo -e '\t<include file="$(find simulator_nimbro)/launch/sender.launch" >'
    echo -e '\t\t<arg name="target" value="'$target'" />'
    echo -e '\t\t<arg name="target_port" value="'$target_port'" />'
    echo -e '\t\t<arg name="ns" value="'$ns'" />'
    echo -e '\t\t<arg name="sender_name" value="'$sender_name'" />'
    echo -e '\t\t<arg name="protocol" value="'$protocol'" />'
    echo -e "\t</include>"
done >> "$LAUNCH_FILE"
echo -e "</launch>" >> "$LAUNCH_FILE"