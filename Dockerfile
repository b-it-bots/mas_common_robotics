FROM argenos/bitbots-base

WORKDIR /kinetic
COPY mas-common.rosinstall /kinetic
RUN rosdep -q update 

RUN wstool init src && wstool merge -t src mas-common.rosinstall
RUN wstool update -t src
RUN rosdep -q install --from-paths src --ignore-src --rosdistro=kinetic -y
