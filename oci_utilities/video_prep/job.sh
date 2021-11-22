#!/usr/bin/env bash
set -o errexit
set -o pipefail
export LANG=C.UTF-8
export LC_ALL=C.UTF-8

script="$(realpath "$0")"
scriptpath="$(dirname "$script")"

# shellcheck source=../includes/check_version.sh
source "$scriptpath/check_version.sh"

# shellcheck source=../includes/set_permissions_job.sh
source "$scriptpath/set_permissions_job.sh"

# shellcheck source=../includes/install_docker.sh
source "$scriptpath/install_docker.sh"

# shellcheck source=../includes/parse_input_subj_no.sh
source "$scriptpath/parse_input_subj_no.sh"

# build docker image
docker build "$scriptpath" --tag bag2video

echo "creating raw data folder"
mkdir -p './data'
echo "downloading files from object storage"
download_result=$(oci os object bulk-download \
    -bn 'rrl-flo-uncompressed' \
    --download-dir './data' \
    --prefix "$subject_padded" \
    --parallel-operations-count 500 \
    --overwrite)
echo "download result: $download_result"

# --- Clean up and upload videos ---

# PODIUM
if [ -d "./data/$subject_padded/ros/podium/" ]
then
    echo 'processing ros podium files'
    echo 'extracting from bags'
    "$scriptpath"/run_docker_bag2vid.sh -d "./data/$subject_padded/ros/podium" -s 90 -v info --audio_topic /robot_audio/audio_relay /lower_realsense/color/image_raw_relay /upper_realsense/color/image_raw_relay
    echo 'transcoding to davinci'
    "$scriptpath"/transcode-to_davinci.sh -t "./data/$subject_padded/ros/podium"
    echo 'uploading to bucket'
    oci os object bulk-upload -bn 'rrl-flo-vids' --src-dir "./data/$subject_padded/ros/podium/transcoded" --prefix "$subject_padded/ros/podium/"
    echo 'done with upload'
else
    echo 'No podium file to process with'
fi

# ROS
if [ -d "./data/$subject_padded/ros/robot/" ]
then
    echo 'processing ros robot files'
    echo 'extracting from bags'
    "$scriptpath"/run_docker_bag2vid.sh -d "./data/$subject_padded/ros/robot" -s 90 -v info --audio_topic /robot_audio/audio_relay /lower_realsense/color/image_raw_relay /upper_realsense/color/image_raw_relay
    echo 'transcoding to davinci'
    "$scriptpath"/transcode-to_davinci.sh -t "./data/$subject_padded/ros/robot"
    echo 'uploading to bucket'
    oci os object bulk-upload -bn 'rrl-flo-vids' --src-dir "./data/$subject_padded/ros/robot/transcoded" --prefix "$subject_padded/ros/podium/"
    echo 'done with upload'
else
    echo 'no robot files to process with'
fi

# ROS (legacy storage)
if ls "./data/$subject_padded/ros/"*.bag &> /dev/null
then
    echo 'processing legacy ros root files'
    echo 'extracting from bags'
    "$scriptpath"/run_docker_bag2vid.sh -d "./data/$subject_padded/ros" -s 90 -v info --audio_topic /robot_audio/audio_relay /lower_realsense/color/image_raw_relay /upper_realsense/color/image_raw_relay
    echo 'transcoding to davinci'
    "$scriptpath"/transcode-to_davinci.sh -t "./data/$subject_padded/ros"
    echo 'uploading to bucket'
    oci os object bulk-upload -bn 'rrl-flo-vids' --src-dir "./data/$subject_padded/ros/transcoded" --prefix "$subject_padded/ros/"
    echo 'done with upload'
else
    echo 'no legacy ros root files to process with'
fi

# Gopro
if ls "./data/$subject_padded/gopro/"*.MP4 &> /dev/null
then
    echo 'processing gopro videos'
    echo 'concatenating'
    "$scriptpath"/concatenate_vids.sh -t "./data/$subject_padded/gopro"
    echo 'transcoding'
    "$scriptpath"/transcode-to_davinci.sh -t "./data/$subject_padded/gopro/concatenated"
    echo 'uploading'
    oci os object bulk-upload -bn 'rrl-flo-vids' --src-dir "./data/$subject_padded/gopro/concatenated/transcoded" --prefix "$subject_padded/gopro/"
    echo 'done'
else
    echo 'no go pro videos to process'
fi

# 3rd-person
if ls "./data/$subject_padded/3rd-person/"*.MP4 &> /dev/null
then
    echo 'processing 3rd-person videos'
    echo 'concatenating'
    "$scriptpath"/concatenate_vids.sh -t "./data/$subject_padded/3rd-person"
    echo 'transcoding'
    "$scriptpath"/transcode-to_davinci.sh -t "./data/$subject_padded/3rd-person/concatenated"
    echo 'uploading'
    oci os object bulk-upload -bn 'rrl-flo-vids' --src-dir "./data/$subject_padded/3rd-person/concatenated/transcoded" --prefix "$subject_padded/3rd-person/"
    echo 'done'
else
    echo 'no 3rd-person videos to process'
fi



echo "job complete"
