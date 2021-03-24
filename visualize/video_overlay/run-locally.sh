#!/bin/bash
set -o errexit
set -o pipefail

# parse options
while getopts :s:c:a:p:d:o: flag
do
    case "${flag}" in
        s) subject=${OPTARG};;
        c) condition=${OPTARG};;
        a) activity=${OPTARG};;
        p) camera=${OPTARG};;
        o) overlay=${OPTARG};;
	      d) data=${OPTARG};;
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
echo "Data directory: $data"
echo "Condition: $condition"
echo "Activity: $activity"
echo "Camera: $camera"

if [[ "$camera" == "all" ]]; then

sudo docker run \
    --mount type=bind,source="$data",target=/code/data \
    --rm \
    video-overlay \
    "data/$condition/$activity" \
    "lower"\
    "$overlay"

sudo docker run \
    --mount type=bind,source="$data",target=/code/data \
    --rm \
    video-overlay \
    "data/$condition/$activity"\
    "upper"\
    "$overlay"
else
sudo docker run \
    --mount type=bind,source="$data",target=/code/data \
    --rm \
    video-overlay\
    "data/$condition/$activity"\
    "$camera"\
    "$overlay"
fi
