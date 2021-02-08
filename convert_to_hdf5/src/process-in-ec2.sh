#!/bin/bash
set -o errexit
set -o pipefail

# parse options
while getopts :s: flag
do
    case "${flag}" in
        s) subject=${OPTARG};;
        :) echo 'missing argument' >&2; exit 1;;
        \?) echo 'invalid option' >&2; exit 1
    esac
done

# check subject number
echo "Passed: $subject"
re='^[0-9]+$'
if ! [[ $subject =~ $re ]] ; then
   echo "error: Subject number is not a number" >&2; exit 1
fi

subject_padded=$(printf '%03d' "$subject")
echo "Processing for subj: $subject_padded"

src=s3://flo-exp-aim1-data-raw/$subject_padded/
meta=s3://flo-exp-aim1-data-meta/$subject_padded/meta.yaml
#src=/media/mjsobrep/flo-external/008/ # <- this should get pulled in from batch job
# copy from s3 into local storage

# note: subfolders: ros, 3rd-person, gopro

#Get the ros files of interest, note that there are some parameter files that we need to ignore, hence the grep
bag_files=$(aws s3 ls "$src"'ros/'| awk '{print $4}' | grep bag.bz2)

mkdir /data/ros/

aws s3 cp "$meta" /data/ros/

for bag_fn in $bag_files
do
    #bring in the bag file from s3
    aws s3 cp "$src"'ros/'"$bag_fn" /data/ros/

    lbzip2 -d "/data/ros/$bag_fn"

    docker run  \
        --mount type=bind,source="/data/ros",target=/data \
        -it \
        --rm \
        hdf5convert \
        roslaunch convert_to_hdf5 convert_to_hdf5.launch bag_file:="/data/${bag_fn%.*}" out_dir:="/data" meta_file:="/data/meta.yaml"

    rm "/data/ros/${bag_fn%.*}"
done

echo "--------------------------------------"
echo "--- Done generating HDF5 files !!!!---"
echo "--------------------------------------"
