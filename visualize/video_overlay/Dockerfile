FROM ubuntu

# Note, this is meant to run on both arm64 and amd64

RUN apt-get update  -y
RUN DEBIAN_FRONTEND="noninteractive" apt-get -y install tzdata
RUN apt-get install -y python3-venv python3-pip lbzip2 curl unzip libhdf5-serial-dev hdf5-tools libhdf5-dev python3-opencv python3-dev python3-numpy pkg-config
RUN pip3 install h5py tqdm

RUN mkdir -p /data

COPY ./overlay_wrists.py  ./

ENTRYPOINT ["./overlay_wrists.py"]
