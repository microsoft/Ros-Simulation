#!/bin/bash

# Copyright (c) Microsoft Corporation. All rights reserved.
# Licensed under the MIT License.

ROBOT_COUNT=${1:-3}
PORT_BASE=${2:-17000}
ROBOT_TARGET_PREFIX=${3:-robot-nimbro-}
ROBOT_PREFIX=${4:-hackbot}
PROTOCOL=${5:-tcp}

SCRIPT_DIR=$(rospack find simulator_nimbro)/scripts 
"$SCRIPT_DIR"/sender.sh "$ROBOT_COUNT" "$ROBOT_TARGET_PREFIX" "$ROBOT_PREFIX" "$PROTOCOL"
"$SCRIPT_DIR"/receiver.sh n="$ROBOT_COUNT" "$PORT_BASE" "$ROBOT_PREFIX" "$PROTOCOL"

roslaunch simulator_nimbro simulator_nimbro.launch