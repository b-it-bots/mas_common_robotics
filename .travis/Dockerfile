FROM bitbots/bitbots-base:kinetic

WORKDIR /kinetic
COPY mas-common.rosinstall /kinetic

RUN wstool init --shallow src && \
    wstool merge -t src mas-common.rosinstall && \
    cd src && wstool remove mas_common_robotics && cd - && \
    wstool update -t src

ADD ./ /kinetic/src/mas_common_robotics

RUN apt-get update -qq && \
    apt-get upgrade -qq && \
    rosdep update -qq && \
    rosdep -q install --from-paths src --ignore-src --rosdistro=kinetic -y && \
    rm -rf /var/lib/apt/lists/*

RUN catkin config --init && \
    catkin config --extend /opt/ros/kinetic && \
    catkin config --install --install-space /opt/ros/mas_stable && \
    catkin build && \
    rm -rf /kinetic/src/*
