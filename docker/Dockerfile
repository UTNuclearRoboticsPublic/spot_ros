ARG ROS_DISTRO=humble
ARG SPOT_VERSION

FROM ros:${ROS_DISTRO}-ros-base AS spot_sdk

SHELL ["/bin/bash", "-l", "-c"]

ENV DEBIAN_FRONTEND=noninteractive
ENV HOME=/root

# Set up terminal and bashrc
RUN sed -i 's/#force_color_prompt=yes/force_color_prompt=yes/' "/root/.bashrc" \
    && sed -i 's/01;32m/01;36m/g; s/01;34m/01;35m/g' "/root/.bashrc"

# Set up a virtual environment to make python install correctly on all version of Ubuntu
WORKDIR /colcon_ws/src
RUN apt update && apt install -y virtualenv && rm -rf /var/lib/apt/lists/*
RUN virtualenv --system-site-packages /spot_venv

# Set up the environment to use venv binaries by default
ENV PATH="/spot_venv/bin:$PATH"

# Install the boston dynamics SDK and other python dependencies
ARG SPOT_VERSION
RUN if [ -z "$SPOT_VERSION" ]; then \
        python3 -m pip install bosdyn-client; \
    else \
        python3 -m pip install bosdyn-client==${SPOT_VERSION}; \
    fi;

CMD bash

# -------------------------------------------------------------------------------------------------
# -------------------------------------------------------------------------------------------------

FROM spot_sdk AS spot_ros

RUN apt update && apt install -y \
        ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
        ros-${ROS_DISTRO}-twist-mux \
        git \
        jq \
        openssh-client

# Add GitHub's key to known_hosts to prevent interactive prompt
RUN mkdir -p -m 0700 ~/.ssh && ssh-keyscan github.com >> ~/.ssh/known_hosts

WORKDIR /colcon_ws/src
RUN git clone -b devel https://github.com/UTNuclearRoboticsPublic/spot_ros.git
RUN git clone -b devel https://github.com/UTNuclearRoboticsPublic/spot_manipulation.git

RUN python3 -m pip install -r spot_ros/requirements.txt yq

WORKDIR /colcon_ws
RUN rosdep update \
    && rosdep install --from-paths src -i -y --skip-keys="gazebo gazebo_ros_pkgs" \
    && rm -rf /var/lib/apt/lists/*

RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc
ENV ROS_DISTRO=${ROS_DISTRO}

ADD entrypoints /entrypoints
RUN chmod +x /entrypoints/*

CMD ["bash"]
