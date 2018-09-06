#!/bin/bash

# Copyright (c) Microsoft Corporation. All rights reserved.
# Licensed under the MIT License.

ARGOS_ROBOT_PREFIX=${1:-hackbot}
ARGOS_NUM_ROBOTS=${2:-3}
ARGOS_NUM_THREADS=${3:-4}
SIMULATOR_CALLBACK_TIMEOUT=${4:-0.1}

export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib/argos3
export ARGOS_PLUGIN_PATH=${CATKIN_WS_PATH}/install/lib/argos3_plugins
export ARGOS_ROBOT_PREFIX
export ARGOS_NUM_ROBOTS
export ARGOS_NUM_THREADS
export SIMULATOR_CALLBACK_TIMEOUT
