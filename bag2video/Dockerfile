FROM ros:noetic

SHELL ["/bin/bash", "-c"]

RUN apt-get update  -y && apt-get install -y python3-pip python3-opencv python3-dev python3-numpy ros-noetic-cv-bridge python3-rosbag ros-noetic-vision-opencv ffmpeg

RUN pip3 install tqdm

RUN ln -s /usr/bin/python3 /usr/bin/python

RUN mkdir -p /data

COPY ./bag2video.py /

RUN echo '#!/bin/bash' >> /entrypoint.sh && \
    echo 'set -e' >> /entrypoint.sh && \
    echo 'source /opt/ros/$ROS_DISTRO/setup.bash' >> /entrypoint.sh && \
    echo 'echo "passed: $@"' >> /entrypoint.sh && \
    echo '/bag2video.py "$@"' >> /entrypoint.sh && \
    chmod u+x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
#ENTRYPOINT ["/ros_install_entry.sh","/bag2video.py","-t","/data"]
