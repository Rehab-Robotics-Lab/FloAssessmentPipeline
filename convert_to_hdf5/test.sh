#Check for data folder in args:
if [ "$1" != "" ]; then
    data="$1"
else
    data=~/'data'
fi

echo "Looking for data at: ${data}"

docker build . --tag hdf5convert
docker run  \
    --mount type=bind,source=$data,target=/data \
    -it \
    --rm \
    hdf5convert
