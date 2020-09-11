FROM ros:noetic

RUN apt-get update  -y
RUN apt-get install -y python3-venv python3-pip
RUN pip3 install h5py

RUN mkdir -p catkin_ws/src/convert_to_hdf5
RUN mkdir -p data

COPY ./launch         catkin_ws/src/convert_to_hdf5/launch
COPY ./src            catkin_ws/src/convert_to_hdf5/src
COPY ./CMakeLists.txt catkin_ws/src/convert_to_hdf5/
COPY ./package.xml    catkin_ws/src/convert_to_hdf5/

WORKDIR catkin_ws
RUN  ./../ros_entrypoint.sh catkin_make
WORKDIR /

RUN  ./ros_entrypoint.sh rosdep install --from-paths /catkin_ws/src/ --ignore-src -r -y

COPY ./ros_entrypoint.sh /

CMD ["roslaunch", "convert_to_hdf5", "main.launch"]
