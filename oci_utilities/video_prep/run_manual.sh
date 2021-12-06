#!/usr/bin/env bash
set -o errexit
set -o pipefail

OCI_CLI_AUTH=instance_principal
export OCI_CLI_AUTH

script="$(realpath "$0")"
scriptpath="$(dirname "$script")"

# shellcheck source=../includes/parse_input_subj_no.sh
source "$scriptpath/parse_input_subj_no.sh"

### download files
rm -rf ./data
mkdir -p ./data
oci os object bulk-download -bn 'rrl-flo-raw' --dest-dir ./data --prefix "$subject_padded"

### uncompress files
echo 'uncompressing files'
find ./data -name '*.bz2' -exec lbzip2 -d -f -n 48 {} \;

### If podium
if [ -d "./data/$subject_padded/ros/podium/" ]
then
echo 'processing podium files'
./video_prep/run_docker_bag2vid.sh -d "$(pwd)/data/$subject_padded/ros/podium" -s 90 -v info --audio_topic /robot_audio/audio_relay /lower_realsense/color/image_raw_relay /upper_realsense/color/image_raw_relay
./video_prep/transcode-to_davinci.sh -t "$(pwd)/data/$subject_padded/ros/podium"
fi

### If robot
if [ -d "./data/$subject_padded/ros/robot/" ]
then
./video_prep/run_docker_bag2vid.sh -d "$(pwd)/data/$subject_padded/ros/robot" -s 90 -v info --audio_topic /robot_audio/audio_relay /lower_realsense/color/image_raw_relay /upper_realsense/color/image_raw_relay
./video_prep/transcode-to_davinci.sh -t "$(pwd)/data/$subject_padded/ros/robot"
fi

### If all together
if ls "./data/$subject_padded/ros/"*.bag &> /dev/null
then
./video_prep/run_docker_bag2vid.sh -d "$(pwd)/data/$subject_padded/ros" -s 90 -v info --audio_topic /robot_audio/audio_relay /lower_realsense/color/image_raw_relay /upper_realsense/color/image_raw_relay
./video_prep/transcode-to_davinci.sh -t "$(pwd)/data/$subject_padded/ros"
fi

### For gopro
./video_prep/concatenate_vids.sh -t "$(pwd)/data/$subject_padded/gopro"
./video_prep/transcode-to_davinci.sh -t "$(pwd)/data/$subject_padded/gopro/concatenated"

### For 3rd-person
./video_prep/concatenate_vids.sh -t "$(pwd)/data/$subject_padded/3rd-person"
./video_prep/transcode-to_davinci.sh -t "$(pwd)/data/$subject_padded/3rd-person/concatenated"

### upload data
oci os object bulk-upload -bn 'rrl-flo-vids' --src-dir "./data/$subject_padded" --include '*.mov' --include '*.MOV' --include '*.mp4' --include '*.MP4' --include '*.avi' --include '*.AVI' --prefix "$subject_padded/"

rm -rf "./data/$subject_padded"
