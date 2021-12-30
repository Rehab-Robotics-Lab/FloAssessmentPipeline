#!/bin/bash
set -o errexit
set -o pipefail

rerun=false

while getopts :d:s:r flag
do
    case "${flag}" in
        d) data=${OPTARG};;
        s) source=${OPTARG};;
        r) rerun=true;;
        :) echo 'missing argument' >&2; exit 1;;
        \?) echo 'invalid option' >&2; exit 1
    esac
done

echo "Processing Files in data folder: $data"

video_file='full_data-vid.hdf5'
novideo_file='full_data-novid.hdf5'
transforms_file='transforms.json'

if $rerun
then
docker run \
    --mount type=bind,source="$data",target=/data \
    --rm \
    -it \
    openpose\
    python3 -m pose_body.scripts.process_hdf5 -v "/data/$video_file" -n "/data/$novideo_file" -t "/data/$transforms_file" -s "$source" -c "lower" --rerun
else
docker run \
    --mount type=bind,source="$data",target=/data \
    --rm \
    -it \
    openpose\
    python3 -m pose_body.scripts.process_hdf5 -v "/data/$video_file" -n "/data/$novideo_file" -t "/data/$transforms_file" -s "$source" -c "lower" --no-rerun
fi
