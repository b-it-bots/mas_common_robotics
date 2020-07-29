FROM bitbots/bitbots-common:melodic

LABEL maintainer="Argentina Ortega"

WORKDIR /travis
COPY mas-common.rosinstall /travis


RUN wstool init --shallow src && \
    wstool merge -t src mas-common.rosinstall && \
    cd src && wstool remove mas_common_robotics && cd - && \
    wstool update -t src

COPY ./ /travis/src/mas_common_robotics

RUN catkin config --init && \
    catkin config --extend /opt/ros/melodic && \
    apt-get update -qq && \
    rosdep update -qq && \
    rosdep -q install --from-paths src --ignore-src --rosdistro=melodic -y && \
    rm -rf /var/lib/apt/lists/* && \
    catkin build --no-status
