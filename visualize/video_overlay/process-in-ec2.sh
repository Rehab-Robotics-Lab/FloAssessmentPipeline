#!/bin/bash
set -o errexit
set -o pipefail

aws configure set default.s3.max_concurrent_requests 25
aws configure set default.s3.max_queue_size 2500

# parse options
while getopts :s:c:a:p: flag
do
    case "${flag}" in
        s) subject=${OPTARG};;
        c) condition=${OPTARG};;
        a) activity=${OPTARG};;
        p) camera=${OPTARG};;
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

if [[ "$camera" =~ ^(upper|lower|all)$ ]]; then
    echo "processing for $camera realsense"
else
    echo "invalid camera parameter passed: $camera"
fi

subject_padded=$(printf '%03d' "$subject")
echo "Processing for subj: $subject_padded"

hdf5=s3://flo-exp-aim1-data-hdf5/$subject_padded/
source_file="$hdf5$condition/$activity.hdf5"
pose_file="$hdf5$condition/$activity-novid.hdf5"

aws s3 cp "$source_file" "/data/$activity.hdf5"
aws s3 cp "$pose_file" "/data/$activity-novid.hdf5"

if [[ "$camera" == "all" ]]; then

docker run \
    --mount type=bind,source="/data",target=/data \
    --rm \
    video-overlay \
    "/data/$activity" \
    "lower"
    aws s3 cp "/data/$activity-lower-wrist.avi" "$hdf5$condition/$activity-lower-wrist.avi"

docker run \
    --mount type=bind,source="/data",target=/data \
    --rm \
    video-overlay \
    "/data/$activity" \
    "upper"
    aws s3 cp "/data/$activity-upper-wrist.avi" "$hdf5$condition/$activity-upper-wrist.avi"
else
docker run \
    --mount type=bind,source="/data",target=/data \
    --rm \
    video-overlay \
    "/data/$activity" \
    "$camera"
    # move video back to folder
    aws s3 cp "/data/$activity-$camera-wrist.avi" "$hdf5$condition/$activity-$camera-wrist.avi"
fi



rm /data/* -f
