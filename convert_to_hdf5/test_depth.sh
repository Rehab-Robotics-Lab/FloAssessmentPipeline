#!/bin/bash
set -o errexit
set -o pipefail



data="/media/gsuveer/391cd01c-d5a2-4313-947a-da8978447a80/gsuveer/Desktop/Flo_data"
bag="/data/flo_recording_2020-12-16-15-47-05_28.bag" 
meta="/data/meta.yaml"
code="/home/gsuveer/catkin_ws/src/LilFloAssessmentPipeline/convert_to_hdf5"


echo "Saving output to: ${data}"
echo "Working with bag file: ${bag}"


if [ "$rebuild" = true ] ; then
    echo 'rebuilding docker image'
    docker build . --tag hdf5convert
fi

docker run  \
    --mount type=bind,source="$data",target=/data \
    --mount type=bind,source="$code",target=/catkin_ws/src/convert_to_hdf5 \
    -it \
    --rm \
    hdf5convert \
    bash

#roslaunch convert_to_hdf5 convert_to_hdf5.launch bag_file:="$bag" out_dir:="/data" meta_file:="$meta"
