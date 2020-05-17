FROM tensorflow/tensorflow:1.14.0-gpu
#FROM ros:melodic-ros-base

#RUN apt-get install -y python-pip
#RUN pip install tensorflow==1.14.0 # might want tensorflow for gpu instead

### ROS ###
# Taken from https://github.com/osrf/docker_images/blob/master/ros/melodic/ubuntu/bionic/ros-core/Dockerfile
# setup timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    apt-get update && \
    apt-get install -q -y --no-install-recommends tzdata && \
    rm -rf /var/lib/apt/lists/*

# install packages
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    dirmngr \
    gnupg2 \
    && rm -rf /var/lib/apt/lists/*

# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# setup sources.list
RUN echo "deb http://packages.ros.org/ros/ubuntu bionic main" > /etc/apt/sources.list.d/ros1-latest.list

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

ENV ROS_DISTRO melodic

# install ros packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-melodic-ros-core=1.4.1-0* \
    && rm -rf /var/lib/apt/lists/*

# setup entrypoint
COPY ./ros_entrypoint.sh /

## ROS from https://github.com/osrf/docker_images/blob/master/ros/melodic/ubuntu/bionic/ros-base/Dockerfile
# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    build-essential \
    python-rosdep \
    python-rosinstall \
    python-vcstools \
    && rm -rf /var/lib/apt/lists/*

# bootstrap rosdep
RUN rosdep init && \
  rosdep update --rosdistro $ROS_DISTRO

# install ros packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-melodic-ros-base=1.4.1-0* \
    && rm -rf /var/lib/apt/lists/*

### Emotion Recognition ###
WORKDIR /home/data
RUN /ros_entrypoint.sh rosdep update

WORKDIR /home/catkin_ws/src/emotion_recognition
COPY ./package.xml ./package.xml

WORKDIR /home/catkin_ws
RUN apt-get update \
    && /ros_entrypoint.sh rosdep install --from-paths ./src --ignore-src -r -y\
    && rm -rf /var/lib/apt/lists/*


COPY ./ ./src/emotion_recognition

RUN /ros_entrypoint.sh catkin_make
RUN sed -i "`wc -l < /ros_entrypoint.sh`i\\source \"/home/catkin_ws/devel/setup.bash\"\\" /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh", "roslaunch","emotion_recognition", "emotion_recognition.launch"]
