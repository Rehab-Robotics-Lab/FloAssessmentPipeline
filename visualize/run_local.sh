#!/bin/bash
set -o errexit
set -o pipefail

script=$(realpath "$0")
scriptpath=$(dirname "$script")

help_message="
Run visualization code to see what is in the dockerfiles

Will expect a directory with files:
 - full_data-vid.hdf5
 - full_data-novid.hdf5

Will generate video files

Args:
 -h: print help
 -d <dir>: path to directory with files (full_data-vid.hdf, full_data-novid.hdf5)
 -c <camera>: the camera to use (lower, upper)
 -f <function>: the function to use (wrists, 2dSkeleton, 3dSkeleton, angular_motion)
 -r: rebuild the docker file that this uses
"

rebuild=false
help=false
# parse options
while getopts :hrd:c:f: flag
do
    case "${flag}" in
        d) data=${OPTARG};;
        c) camera=${OPTARG};;
        f) overlay=${OPTARG};;
        r) rebuild=true;;
        h) help=true;;
        :) echo 'missing argument' >&2; exit 1;;
        \?) echo 'invalid option' >&2; exit 1
    esac
done

if [ "$rebuild" = true ] ; then
    echo 'rebuilding docker image'
    docker build -t video-overlay -f "$scriptpath/../dockerfiles/video_overlay" "$scriptpath/../"
fi

if [ "$help" = true ] ; then
    echo "$help_message"
    echo "----------------"
    echo "Help from package:"
    docker run \
        --rm \
        -it \
        video-overlay\
        python3 -m visualize.visualize -h
    exit 0
fi

if [[ "$camera" == "all" ]]; then

docker run \
    --mount type=bind,source="$data",target=/data \
    --rm \
    -it \
    video-overlay \
    "/data/" \
    "lower"\
    "$overlay"

docker run \
    --mount type=bind,source="$data",target=/data \
    --rm \
    -it \
    video-overlay \
    "/data/"\
    "upper"\
    "$overlay"
else
docker run \
    --mount type=bind,source="$data",target=/data \
    --rm \
    -it \
    video-overlay\
    python3 -m visualize.visualize --dir "/data/" --cam "$camera" --function "$overlay"
fi
