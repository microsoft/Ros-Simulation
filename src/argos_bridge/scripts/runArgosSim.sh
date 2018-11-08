#!/bin/bash

# Copyright (c) Microsoft Corporation. All rights reserved.
# Licensed under the MIT License.

ARGOS_WORLD_FILE=${1:-construct.argos}

parent_path=$( cd "$(dirname "${BASH_SOURCE[0]}")" || exit; pwd -P )

cd "$parent_path" || exit

# Apply environment variables to ARGoS world file template to create new world file
envsubst < ../argos_worlds/${ARGOS_WORLD_FILE} > ../argos_worlds/launch.tmp.argos
argos3 -c ../argos_worlds/launch.tmp.argos
