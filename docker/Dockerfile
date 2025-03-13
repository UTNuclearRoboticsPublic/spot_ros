ARG ROS_DISTRO
ARG SPOT_VERSION

FROM ros:${ROS_DISTRO}-ros-base AS spot_sdk

ENV DEBIAN_FRONTEND=noninteractive
ENV HOME=/root

# Set up terminal and bashrc
RUN sed -i 's/#force_color_prompt=yes/force_color_prompt=yes/' "/root/.bashrc" \
    && sed -i 's/01;32m/01;36m/g; s/01;34m/01;35m/g' "/root/.bashrc"

# Install the boston dynamics SDK and other python dependencies
ARG SPOT_VERSION
RUN apt update \
    && apt install python3 python3-pip \
    && \
    if [ -z "$SPOT_VERSION" ]; then \
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
        git \
        openssh-client

# Add GitHub's key to known_hosts to prevent interactive prompt
RUN mkdir -p -m 0700 ~/.ssh && ssh-keyscan github.com >> ~/.ssh/known_hosts

WORKDIR /colcon_ws/src
RUN --mount=type=ssh git clone git@github.com:UTNuclearRobotics/nrg_spot_manipulation.git -b devel
RUN --mount=type=ssh git clone git@github.com:UTNuclearRobotics/spot_ros.git -b devel

RUN python3 -m pip install -r spot_ros/requirements.txt 

WORKDIR /colcon_ws
RUN rosdep update \
    && rosdep install --from-paths src -i -y --skip-keys="gazebo gazebo_ros_pkgs" \
    && rm -rf /var/lib/apt/lists/*

SHELL ["/bin/bash", "-l", "-c"]
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

ADD entrypoints /entrypoints
RUN chmod +x /entrypoints/*

# -------------------------------------------------------------------------------------------------
# -------------------------------------------------------------------------------------------------

FROM spot_ros AS bringup
ENTRYPOINT [ "/entrypoints/bringup.bash" ]
