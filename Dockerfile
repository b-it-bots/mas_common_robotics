FROM bitbots/bitbots-base:kinetic

WORKDIR /kinetic
COPY mas-common.rosinstall /kinetic

RUN wstool init --shallow src && \
    wstool merge -t src mas-common.rosinstall && \
    wstool update -t src

ADD . /kinetic/src/mas_common_robotics

RUN apt-get update -qq && \
    rosdep update && \
    rosdep -q install --from-paths src --ignore-src --rosdistro=kinetic -y && \
    rm -rf /var/lib/apt/lists/*

RUN catkin config --init && \
    catkin config --extend /opt/ros/kinetic && \
    catkin config --install --install-space /opt/ros/mas_stable && \
    catkin build
