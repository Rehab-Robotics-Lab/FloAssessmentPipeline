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
 -d <dir>: path to directory with files
 -s <src>: the source the data comes from (robot, podium, or mixed).
 -r: rebuild the docker file that this uses
"

rebuild=false
# parse options
while getopts :hrd:c:o: flag
do
    case "${flag}" in
        d) data=${OPTARG};;
        c) camera=${OPTARG};;
        o) overlay=${OPTARG};;
        r) rebuild=true;;
        h) echo "$help_message";;
        :) echo 'missing argument' >&2; exit 1;;
        \?) echo 'invalid option' >&2; exit 1
    esac
done

if [ "$rebuild" = true ] ; then
    echo 'rebuilding docker image'
    docker build -t video-overlay -f "$scriptpath/../../dockerfiles/video_overlay" "$scriptpath/../../"
fi

if [[ "$camera" == "all" ]]; then

docker run \
    --mount type=bind,source="$data",target=/data \
    --rm \
    video-overlay \
    "/data" \
    "lower"\
    "$overlay"

docker run \
    --mount type=bind,source="$data",target=/data \
    --rm \
    video-overlay \
    "/data"\
    "upper"\
    "$overlay"
else
docker run \
    --mount type=bind,source="$data",target=/data \
    --rm \
    video-overlay\
    "/data"\
    "$camera"\
    "$overlay"
fi
