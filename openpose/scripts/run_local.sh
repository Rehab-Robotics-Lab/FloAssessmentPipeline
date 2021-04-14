#!/bin/bash
set -o errexit
set -o pipefail

data="/media/gsuveer/391cd01c-d5a2-4313-947a-da8978447a80/gsuveer/Desktop/Flo_data"

while getopts :d: flag
do
    case "${flag}" in
        d) data=${OPTARG};;
        :) echo 'missing argument' >&2; exit 1;;
        \?) echo 'invalid option' >&2; exit 1
    esac
done

echo "Processing Files in data folder: $data"

docker run \
    --mount type=bind,source="$data",target=/data \
    --rm \
    -it \
    openpose

