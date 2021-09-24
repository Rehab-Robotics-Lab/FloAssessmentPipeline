#!/bin/bash
set -o errexit
set -o pipefail

# parse options
rebuild=false

while getopts :d:b:r flag
do
    case "${flag}" in
        d) data=${OPTARG};;
        b) bag=${OPTARG};;
        r) rebuild=true;;
        :) echo 'missing argument' >&2; exit 1;;
        \?) echo 'invalid option' >&2; exit 1
    esac
done

data="${data%/}"
bag="/data/$bag"

echo "Saving output to: ${data}"
echo "Working with bag file: ${bag}"

if [ "$rebuild" = true ] ; then
    echo 'rebuilding docker image'
    docker build . --tag hdf5convert
fi

docker run  \
    --mount type=bind,source="$data",target=/data \
    -it \
    --rm \
    hdf5convert \
    roslaunch convert_to_hdf5 convert_to_hdf5.launch bag_file:="$bag" out_dir:="/data"
