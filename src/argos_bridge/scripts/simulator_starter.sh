#!/bin/bash

# Copyright (c) Microsoft Corporation. All rights reserved.
# Licensed under the MIT License.

ROBOT_COUNT=${1:-3}
SIMULATOR_THREAD_COUNT=${2:-4}
ROBOT_PREFIX=${3:-hackbot}
SIMULATOR_CALLBACK_TIMEOUT=${4:-0.1}
ARGOS_WORLD_FILE=${5:-construct.argos}

/root/scripts/vncserver-startup.sh 

# shellcheck disable=SC1091
source /root/catkin_ws/devel/setup.bash

# shellcheck disable=SC1091
source /root/catkin_ws/src/argos_bridge/scripts/setupArgosEnv.sh "$ROBOT_PREFIX" "$ROBOT_COUNT" "$SIMULATOR_THREAD_COUNT" "$SIMULATOR_CALLBACK_TIMEOUT"
/root/catkin_ws/src/argos_bridge/scripts/runArgosSim.sh "${ARGOS_WORLD_FILE}"
