# Copyright (c) Microsoft Corporation. All rights reserved.
# Licensed under the MIT License.

FROM ros:kinetic-ros-core
 
RUN apt-get update && apt-get install -y --no-install-recommends \
    cmake qt5-default libfreeimage-dev libfreeimageplus-dev \
    freeglut3-dev libxi-dev libxmu-dev liblua5.2-dev \
    lua5.2-dev doxygen graphviz graphviz-dev libgsl-dev \
  && rm -rf /var/lib/apt/lists/*

## Compile the ARGoS simulator
RUN git clone https://github.com/xinyiou/argos3.git /argos3 \
    && cd /argos3 \
    && git checkout 83dcc4d0d09ef706651c590be4830ffc2f6e850b \
    && mkdir /argos3/build_simulator \
    && cd /argos3/build_simulator \
    && cmake -DCMAKE_BUILD_TYPE=Debug \
        -DCMAKE_INSTALL_PREFIX=/usr/local \
        -DARGOS_BUILD_FOR=simulator \
        -DARGOS_BUILD_NATIVE=OFF \
        -DARGOS_THREADSAFE_LOG=ON \
        -DARGOS_DYNAMIC_LIBRARY_LOADING=ON \
        -DARGOS_USE_DOUBLE=ON \
        -DARGOS_DOCUMENTATION=OFF \
        -DARGOS_INSTALL_LDSOCONF=ON \
        ../src \
    && make install \
    && ldconfig \
    && rm -rf /argos3
 
CMD ["/bin/bash", "-c", "argos3 -q all"]