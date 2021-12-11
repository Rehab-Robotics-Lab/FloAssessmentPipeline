FROM ros:noetic

# Note, this is meant to run on both arm64 and amd64

RUN apt-get update  -y
RUN apt-get install -y python3-venv python3-pip lbzip2 curl unzip libhdf5-serial-dev hdf5-tools libhdf5-dev python3-opencv python3-dev python3-numpy ros-noetic-cv-bridge ros-noetic-rospy-message-converter
RUN pip3 install h5py tqdm
RUN curl "https://awscli.amazonaws.com/awscli-exe-linux-x86_64.zip" -o "awscliv2.zip" && unzip awscliv2.zip && ./aws/install

RUN mkdir -p catkin_ws/src/convert_to_hdf5
RUN mkdir -p /data

COPY ./CMakeLists.txt catkin_ws/src/convert_to_hdf5/
COPY ./package.xml    catkin_ws/src/convert_to_hdf5/

WORKDIR catkin_ws
RUN  ./../ros_entrypoint.sh catkin_make
WORKDIR /

COPY ./launch         catkin_ws/src/convert_to_hdf5/launch
COPY ./src            catkin_ws/src/convert_to_hdf5/src

#RUN  ./ros_entrypoint.sh rosdep install --from-paths /catkin_ws/src/ --ignore-src -r -y

COPY ./ros_entrypoint.sh /

CMD ["roslaunch", "convert_to_hdf5", "main.launch"]
