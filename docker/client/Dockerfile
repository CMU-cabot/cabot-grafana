FROM ros:humble AS base

RUN apt update && apt install --no-install-recommends -y \
    curl \
    python3-pip \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-tf-transformations \
    ros-humble-cv-bridge \
    && \	
    rm -rf /var/lib/apt/lists/*

RUN pip3 install \
    influxdb-client \
    transforms3d \
    pyproj

RUN apt update && apt install -y \
    g++ \
    cmake \
    libproj-dev \
    libcurl4-openssl-dev \
    libboost1.74-dev \
    libboost-system1.74-dev \
    && \	
    rm -rf /var/lib/apt/lists/*
 
RUN cd /tmp && \
    git clone https://github.com/libcpr/cpr.git -b 1.10.5 && \
    cd cpr && mkdir build && cd build && \
    cmake .. -DCPR_USE_SYSTEM_CURL=ON && \
    cmake --build . --parallel && \
    sudo cmake --install . && \
    git clone https://github.com/offa/influxdb-cxx && \
    mkdir -p influxdb-cxx/build && \
    cd influxdb-cxx/build && \
    cmake -D INFLUXCXX_TESTING:BOOL=OFF .. && \
    make install

ENV LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH

COPY launch.sh /launch.sh


FROM base as cache

ENV UNDERLAY_WS=/opt/underlay_ws
RUN mkdir -p $UNDERLAY_WS/src
COPY --from=cabot_src ./cabot_influxdb_client $UNDERLAY_WS/src/cabot_influxdb_client
COPY --from=cabot_src ./cabot-common/cabot_msgs $UNDERLAY_WS/src/cabot_msgs

WORKDIR $UNDERLAY_WS

# copy package.xml files with parent directories for cache
RUN find . -name "src" -type d \
      -mindepth 1 -maxdepth 2 -printf '%P\n' \
      | xargs -I % mkdir -p /tmp/opt/% && \
    find . -name "package.xml" \
      | xargs cp --parents -t /tmp/opt && \
    find . -name "COLCON_IGNORE" \
      | xargs cp --parents -t /tmp/opt || true

RUN apt update && rosdep update && rosdep install -q -y \
    --from-paths src \
    --ignore-src

RUN . /opt/ros/humble/setup.sh && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

FROM base AS build
ENV ROS_DISTRO=humble
ARG TZ=UTC
ENV TZ=$TZ
RUN sudo ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ | sudo tee /etc/timezone

ARG ROS_DISTRO

ENV UNDERLAY_WS=/opt/underlay_ws

WORKDIR $UNDERLAY_WS

COPY --from=cache $UNDERLAY_WS/install $UNDERLAY_WS/install
# copy package.xml files from cache
COPY --from=cache /tmp/opt/src $UNDERLAY_WS/src
COPY ros_entrypoint.sh /ros_entrypoint.sh

RUN apt update && \
    apt install -y gosu tree \
    && \	
    rm -rf /var/lib/apt/lists/*

### make user environment

ENV USERNAME developer
# Replace 1000 with your user/group id
ARG UID=1000
RUN useradd -m $USERNAME && \
        echo "$USERNAME:$USERNAME" | chpasswd && \
        usermod --shell /bin/bash $USERNAME && \
        usermod -aG sudo $USERNAME && \
        mkdir -p /etc/sudoers.d/ && \
        echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/$USERNAME && \
        chmod 0440 /etc/sudoers.d/$USERNAME && \
        usermod  --uid $UID $USERNAME && \
        groupmod --gid $UID $USERNAME

ENV HOME /home/$USERNAME

RUN mkdir $HOME/ros2_ws

WORKDIR $HOME/ros2_ws

COPY --from=cabot_src ./docker/home/ros2_ws/launch.sh $HOME/ros2_ws/launch.sh
