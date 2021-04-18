FROM osrf/ros:foxy-desktop

RUN mkdir -p /home/rosusr/dev_ws/src

# Fresh apt
RUN apt update
#RUN apt install vim
#RUN apt install python3-pip
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

RUN sudo mkdir /home/olepor/dev_ws/log
RUN sudo mkdir /home/olepor/dev_ws/install
RUN sudo mkdir /home/olepor/dev_ws/build
RUN sudo chown olepor:olepor /home/olepor/dev_ws/{build,log,install}

# TODO - or something similar
#ADD https://github.com/oleorhagen/steer_bot.git /home/rosuser/catkin_ws/steer_bot

#RUN apt-get update && apt-get upgrade -yy

# Entrypoint should be /bin/sh or bash by default
