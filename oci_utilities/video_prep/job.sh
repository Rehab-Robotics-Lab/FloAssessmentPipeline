#!/usr/bin/env bash
set -o errexit
set -o pipefail

script="$(realpath "$0")"
scriptpath="$(dirname "$script")"

# shellcheck source=../includes/check_version.sh
source "$scriptpath/check_version.sh"

# shellcheck source=../includes/set_permissions.sh
source "$scriptpath/set_permissions.sh"

# shellcheck source=../includes/parse_input_subj_no.sh
source "$scriptpath/parse_input_subj_no.sh"

# shellcheck source=../includes/get_instance_info.sh
source "$scriptpath/get_instance_info.sh"

# shellcheck source=../includes/get_block_volume_id.sh
source "$scriptpath/get_block_volume_id.sh"

# shellcheck source=../includes/attach_block_volume.sh
source "$scriptpath/attach_block_volume.sh"

# shellcheck source=../includes/mount_block_volume.sh
source "$scriptpath/mount_block_volume.sh"

sleep 5

# build docker image
docker build "$scriptpath" --tag bag2video

# --- Clean up and upload videos ---

# PODIUM
if [ -d '/mnt/subj-data/raw/ros/podium/' ]
then
    echo 'processing ros podium files'
    echo 'extracting from bags'
    "$scriptpath"/run_docker_bag2vid.sh -d /mnt/subj-data/raw/ros/podium -s 90 -v info --audio_topic /robot_audio/audio_relay /lower_realsense/color/image_raw_relay /upper_realsense/color/image_raw_relay
    echo 'transcoding to davinci'
    "$scriptpath"/transcode-to_davinci.sh -t /mnt/subj-data/raw/ros/podium
    echo 'uploading to bucket'
    oci os object bulk-upload -bn 'rrl-flo-vids' --src-dir /mnt/subj-data/raw/ros/podium/transcoded --prefix "$subject_padded/ros/podium/"
    echo 'done with upload'
else
    echo 'No podium file to process with'
fi

# ROS
if [ -d '/mnt/subj-data/raw/ros/robot/' ]
then
    echo 'processing ros robot files'
    echo 'extracting from bags'
    "$scriptpath"/run_docker_bag2vid.sh -d /mnt/subj-data/raw/ros/robot -s 90 -v info --audio_topic /robot_audio/audio_relay /lower_realsense/color/image_raw_relay /upper_realsense/color/image_raw_relay
    echo 'transcoding to davinci'
    "$scriptpath"/transcode-to_davinci.sh -t /mnt/subj-data/raw/ros/robot
    echo 'uploading to bucket'
    oci os object bulk-upload -bn 'rrl-flo-vids' --src-dir /mnt/subj-data/raw/ros/robot/transcoded --prefix "$subject_padded/ros/podium/"
    echo 'done with upload'
else
    echo 'no robot files to process with'
fi

# ROS (legacy storage)
if ls /mnt/subj-data/raw/ros/*.bag &> /dev/null
then
    echo 'processing legacy ros root files'
    echo 'extracting from bags'
    "$scriptpath"/run_docker_bag2vid.sh -d /mnt/subj-data/raw/ros -s 90 -v info --audio_topic /robot_audio/audio_relay /lower_realsense/color/image_raw_relay /upper_realsense/color/image_raw_relay
    echo 'transcoding to davinci'
    "$scriptpath"/transcode-to_davinci.sh -t /mnt/subj-data/raw/ros
    echo 'uploading to bucket'
    oci os object bulk-upload -bn 'rrl-flo-vids' --src-dir /mnt/subj-data/raw/ros/transcoded --prefix "$subject_padded/ros/"
    echo 'done with upload'
else
    echo 'no legacy ros root files to process with'
fi

# Gopro
if ls /mnt/subj-data/gopro/*.MP4 &> /dev/null
then
    echo 'processing gopro videos'
    echo 'concatenating'
    "$scriptpath"/concatenate_vids.sh -t /mnt/subj-data/gopro
    echo 'transcoding'
    "$scriptpath"/transcode-to_davinci.sh -t /mnt/subj-data/gopro/concatenated
    echo 'uploading'
    oci os object bulk-upload -bn 'rrl-flo-vids' --src-dir /mnt/subj-data/raw/ros/transcoded --prefix "$subject_padded/ros/"
    echo 'done'
else
    echo 'no go pro videos to process'
fi

# 3rd-person
if ls /mnt/subj-data/3rd-person/*.MP4 &> /dev/null
then
    echo 'processing 3rd-person videos'
    echo 'concatenating'
    "$scriptpath"/concatenate_vids.sh -t /mnt/subj-data/3rd-person
    echo 'transcoding'
    "$scriptpath"/transcode-to_davinci.sh -t /mnt/subj-data/3rd-person/concatenated
    echo 'uploading'
    oci os object bulk-upload -bn 'rrl-flo-vids' --src-dir /mnt/subj-data/raw/ros/transcoded --prefix "$subject_padded/ros/"
    echo 'done'
else
    echo 'no 3rd-person videos to process'
fi



echo "job complete"