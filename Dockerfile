FROM osrf/ros:foxy-desktop

RUN mkdir -p /home/rosusr/dev_ws/src

# Fresh apt
RUN apt update
RUN DEBIAN_FRONTEND=noninteractive apt install -yyyq vim python3-pip ros-foxy-xacro ros-foxy-joint-state-publisher-gui
# RUN apt install python3-pip
#RUN sudo apt upgrade

# Packages currently required
# ros-foxy-xacro
# ros-foxy-joint-state-publisher-gui

# User setup
ARG USER_ID
ARG GROUP_ID

RUN addgroup --gid $GROUP_ID olepor
RUN echo 'olepor ALL=(ALL) NOPASSWD: ALL' >> /etc/sudoers
RUN adduser --disabled-password --gecos '' --uid $USER_ID --gid $GROUP_ID olepor
USER olepor

RUN sudo mkdir -p /home/olepor/dev_ws/log
RUN sudo mkdir -p /home/olepor/dev_ws/install
RUN sudo mkdir -p /home/olepor/dev_ws/build
RUN sudo chown olepor:olepor /home/olepor/dev_ws/log
RUN sudo chown olepor:olepor /home/olepor/dev_ws/install
RUN sudo chown olepor:olepor /home/olepor/dev_ws/build

# TODO - or something similar
#ADD https://github.com/oleorhagen/steer_bot.git /home/rosuser/catkin_ws/steer_bot

#RUN apt-get update && apt-get upgrade -yy

# Entrypoint should be /bin/sh or bash by default
