docker build . --tag hdf5convert
data=~/'data'
docker run  \
    --mount type=bind,source=$data,target=/data \
    -it \
    hdf5convert \
    bash
    #-c "roslaunch convert_to_hdf5 main.launch"
