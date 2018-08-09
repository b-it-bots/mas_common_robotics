FROM argenos/bitbots-base

RUN rosdep update

ADD ./ /kinetic/src/mas_common_robotics
WORKDIR /kinetic

RUN wstool init src && wstool merge -t src src/mas_common_robotics/.travis/travis.rosinstall
RUN wstool update -t src
RUN rosdep -q install --from-paths src --ignore-src --rosdistro=kinetic -y
