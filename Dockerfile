FROM bitbots/bitbots-base:melodic

LABEL maintainer="Argentina Ortega"

WORKDIR /melodic
COPY mas-common.rosinstall /melodic

RUN wstool init --shallow src && \
    wstool merge -t src mas-common.rosinstall && \
    cd src && wstool remove mas_common_robotics && cd - && \
    wstool update -t src

COPY . /melodic/src/mas_common_robotics

RUN apt-get update -qq && \
    rosdep update && \
    rosdep -q install --from-paths src --ignore-src --rosdistro=melodic -y && \
    rm -rf /var/lib/apt/lists/*

RUN catkin config --init && \
    catkin config --extend /opt/ros/melodic && \
    catkin config --install --install-space /opt/ros/mas_stable && \
    catkin build && \
    echo "source /opt/ros/mas_stable/setup.bash" >> ~/.bashrc && \
    rm -rf /melodic/

WORKDIR /
COPY ./.docker/ros_entrypoint.sh /
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
