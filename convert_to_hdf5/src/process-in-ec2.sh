#!/bin/bash
set -o errexit
set -o pipefail

aws configure set default.s3.max_concurrent_requests 1000
aws configure set default.s3.max_queue_size 100000

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
hdf5=s3://flo-exp-aim1-data-hdf5/$subject_padded/
#src=/media/mjsobrep/flo-external/008/ # <- this should get pulled in from batch job
# copy from s3 into local storage

# note: subfolders: ros, 3rd-person, gopro

#Get the ros files of interest, note that there are some parameter files that we need to ignore, hence the grep
bag_files=$(aws s3 ls "$src"'ros/'| awk '{print $4}' | grep bag.bz2)

mkdir /data/ros/

aws s3 cp "$meta" /data/ros/

done_first=false
previous_bag_fn=''
for bag_fn in $bag_files
do
    #bring in the bag file from s3
    aws s3 cp "$src"'ros/'"$bag_fn" /data/ros/

    lbzip2 -d "/data/ros/$bag_fn"

    if [ $done_first = true ] ; then
        docker wait hdf5-converter
        rm "/data/ros/${previous_bag_fn%.*}"
    fi

    docker run  \
        --mount type=bind,source="/data/ros",target=/data \
        -dit \
        --rm \
        --name=hdf5-converter
        hdf5convert \
        roslaunch convert_to_hdf5 convert_to_hdf5.launch bag_file:="/data/${bag_fn%.*}" out_dir:="/data" meta_file:="/data/meta.yaml"

    previous_bag_fn=$bag_fn
    done_first=true
done

if [ $done_first = true ] ; then
    docker wait hdf5-converter
    rm "/data/ros/${previous_bag_fn%.*}"
fi


rm /data/ros/meta.yaml

prior=$(pwd)
cd /data/ros
files2transfer=$(find . -type f)
for fn in $files2transfer
do
    aws s3 cp "$fn" "$hdf5/$(dirname "$fn")" &
done
wait
echo "Done with upload"
cd "$prior"
rm -rf /data/*

echo "--------------------------------------"
echo "--- Done generating HDF5 files !!!!---"
echo "--------------------------------------"
