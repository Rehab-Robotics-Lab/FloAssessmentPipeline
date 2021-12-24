#!/bin/bash
set -o errexit
set -o pipefail

aws configure set default.s3.max_concurrent_requests 25
aws configure set default.s3.max_queue_size 2500

# parse options
while getopts :s:c:a: flag
do
    case "${flag}" in
        s) subject=${OPTARG};;
        c) condition=${OPTARG};;
        a) activity=${OPTARG};;
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

hdf5=s3://flo-exp-aim1-data-hdf5/$subject_padded/
source_file="$hdf5$condition/$activity.hdf5"

aws s3 cp "$source_file" /data/

docker run \
    --mount type=bind,source="/data",target=/data \
    --rm \
    openpose

# move file back to folder
aws s3 cp "/data/$activity-novid.hdf5" "$hdf5$condition/$activity-novid.hdf5"

rm /data/* -f
