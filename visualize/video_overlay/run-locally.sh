#!/bin/bash
set -o errexit
set -o pipefail

# parse options
while getopts :r:c:o: flag
do
    case "${flag}" in
        r) root=${OPTARG};;
        c) camera=${OPTARG};;
        o) overlay=${OPTARG};;
        :) echo 'missing argument' >&2; exit 1;;
        \?) echo 'invalid option' >&2; exit 1
    esac
done

data=$(pathname "$root")
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
