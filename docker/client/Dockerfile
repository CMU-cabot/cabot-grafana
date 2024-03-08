FROM ros:humble

RUN apt update && apt install --no-install-recommends -y \
	curl \
	python3-pip \
	ros-humble-rmw-cyclonedds-cpp \
	ros-humble-tf-transformations \
	ros-humble-cv-bridge
#	&& \
#	rm -rf /var/lib/apt/lists/*

RUN pip3 install \
	influxdb-client \
	transforms3d \
	pyproj

RUN apt install -y \
     g++ \
	 cmake \
	 libcurl4-openssl-dev \
	 libboost1.74-dev \
	 libboost-system1.74-dev

RUN cd /tmp && \
	git clone https://github.com/libcpr/cpr.git && \
	cd cpr && mkdir build && cd build && \
	cmake .. -DCPR_USE_SYSTEM_CURL=ON && \
	cmake --build . --parallel && \
	sudo cmake --install . && \
	git clone https://github.com/offa/influxdb-cxx && \
	mkdir -p influxdb-cxx/build && \
	cd influxdb-cxx/build && \
	cmake -D INFLUXCXX_TESTING:BOOL=OFF .. && \
	make install


COPY launch.sh /launch.sh

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

USER $USERNAME

ENV HOME /home/$USERNAME

RUN mkdir $HOME/ros2_ws

WORKDIR $HOME/ros2_ws
