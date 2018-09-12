FROM bitbots/bitbots-base:kinetic

LABEL maintainer="Argentina Ortega"

WORKDIR /kinetic
COPY mas-common.rosinstall /kinetic

RUN wstool init --shallow src && \
    wstool merge -t src mas-common.rosinstall && \
    cd src && wstool remove mas_common_robotics && cd - && \
    wstool update -t src

COPY . /kinetic/src/mas_common_robotics

RUN apt-get update -qq && \
    rosdep update && \
    rosdep -q install --from-paths src --ignore-src --rosdistro=kinetic -y && \
    rm -rf /var/lib/apt/lists/*

RUN catkin config --init && \
    catkin config --extend /opt/ros/kinetic && \
    catkin config --install --install-space /opt/ros/mas_stable && \
    catkin build && \
    echo "source /opt/ros/mas_stable/setup.bash" >> ~/.bashrc && \
    rm -rf /kinetic/

WORKDIR /
COPY ./.docker/ros_entrypoint.sh /
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
