FROM ros:melodic-ros-base

RUN apt-get update -y
RUN apt-get install -y python-pip
RUN pip install tensorflow==1.14.0 # might want tensorflow for gpu instead

WORKDIR /home/data

WORKDIR /home/catkin_ws/src/emotion_recognition
COPY ./ ./

WORKDIR /home/catkin_ws
RUN /ros_entrypoint.sh catkin_make
RUN sed -i "`wc -l < /ros_entrypoint.sh`i\\source \"/home/catkin_ws/devel/setup.bash\"\\" /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh", "roslaunch","emotion_recognition", "main.launch"]
