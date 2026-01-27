ARG TARGET_ROS_DISTRO=humble
FROM nrg/spot_bringup:${TARGET_ROS_DISTRO} AS base

WORKDIR /colcon_ws/src
COPY ./spot_core ./spot_core
COPY ./spot_manipulation ./spot_manipulation

WORKDIR /colcon_ws
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

COPY ./docker/entrypoints /entrypoints
RUN chmod +x /entrypoints/*

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc \
    && echo "source /colcon_ws/install/local_setup.bash" >> ~/.bashrc \
    && echo "PS1='\[\e[1;33m\][spot]\[\e[0m\]\[\e[1;32m\]\u@\h\[\e[0m\]:\[\e[1;34m\]\w\[\e[0m\]\$ '" >> ~/.bashrc

COPY ./docker/misc /misc
RUN chmod +x /misc/*
RUN echo "alias check_battery='/misc/check_battery.sh'" >> ~/.bashrc
