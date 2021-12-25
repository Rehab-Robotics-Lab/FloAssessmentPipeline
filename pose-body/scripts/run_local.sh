#!/bin/bash
set -o errexit
set -o pipefail

while getopts :d:s: flag
do
    case "${flag}" in
        d) data=${OPTARG};;
        s) source=${OPTARG};;
        :) echo 'missing argument' >&2; exit 1;;
        \?) echo 'invalid option' >&2; exit 1
    esac
done

echo "Processing Files in data folder: $data"

video_file='full_data-vid.hdf5'
novideo_file='full_data-novid.hdf5'
transforms_file='transforms.json'

docker run \
    --mount type=bind,source="$data",target=/data \
    --rm \
    -it \
    openpose\
    ./process_hdf5.py -v "/data/$video_file" -n "/data/$novideo_file" -t "/data/$transforms_file" -s "$source" -c "lower" --rerun
