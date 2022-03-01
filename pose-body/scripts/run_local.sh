#!/bin/bash
set -o errexit
set -o pipefail

script=$(realpath "$0")
scriptpath=$(dirname "$script")

help_message="
Run open pose processing on hdf files.

Will expect a directory with files:
 - full_data-vid.hdf5
 - full_data-novid.hdf5
 - transforms.json

Puts discovered poses into files

Args:
 -h: print help
 -d <dir>: path to directory with files
 -s <src>: the source the data comes from (robot, podium, or mixed).
 -r: rebuild the docker file that this uses
"

while getopts :hrd:s: flag
do
    case "${flag}" in
        d) data=${OPTARG};;
        s) source=${OPTARG};;
        h) echo "$help_message"; exit 0;;
        r) rebuild=true;;
        :) echo 'missing argument' >&2; exit 1;;
        \?) echo 'invalid option' >&2; exit 1
    esac
done

if [ "$rebuild" = true ] ; then
    echo 'rebuilding docker image'
    docker build -t openpose -f "$scriptpath/../../dockerfiles/openpose" "$scriptpath/../../"
fi

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
