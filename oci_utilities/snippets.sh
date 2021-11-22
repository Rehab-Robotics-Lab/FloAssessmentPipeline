#!/bin/bash

## Not meant to be run. just some snippets that are useful for running code
sudo dnf install -y tmux

OCI_CLI_AUTH=instance_principal
export OCI_CLI_AUTH

sudo dnf -y install oraclelinux-developer-release-el8
sudo dnf -y install python36-oci-cli

PROJECT_COMPARTMENT_OCID='ocid1.compartment.oc1..aaaaaaaadznuoh3ntsva2jsj453wwmemd4t2k5rnwniuzkliq7evffxgprua'
export PROJECT_COMPARTMENT_OCID

mkdir -p manual_jobs
oci os object get -bn 'rrl-flo-run' --file 'manual_jobs/run.sh' --name 'manual_jobs/run.sh'

oci-token os object put -bn 'rrl-flo-run' --file 'run.sh' --name 'manual_jobs/run.sh'

oci-token os object put -bn 'rrl-flo-run' --file 'run.sh' --name 'manual_jobs/run.sh'


# https://docs.docker.com/engine/install/centos/

#####################################################
#####################################################
# setup videos
## on local machine
./oci_utilities/video_prep/upload.sh
# -----------
## on vm server
### expand filesystem
sudo /usr/libexec/oci-growfs -y

### install oci-cli
sudo dnf -y install oraclelinux-developer-release-el8
sudo dnf -y install python36-oci-cli

### set permissions
OCI_CLI_AUTH=instance_principal
export OCI_CLI_AUTH

### pull in scripts
oci os object bulk-download -bn 'rrl-flo-run' --download-dir '.' --overwrite

### add in secondary repos
./video_prep/add_extended_repos.sh

### install tmux
sudo dnf install -y tmux

### install lbzip2
sudo dnf install -y lbzip2

### install ffmpeg
sudo dnf -y install https://download.fedoraproject.org/pub/epel/epel-release-latest-8.noarch.rpm
sudo yum-config-manager --enable ol8_codeready_builder
sudo dnf -y localinstall --nogpgcheck https://download1.rpmfusion.org/free/el/rpmfusion-free-release-8.noarch.rpm
sudo dnf -y install --nogpgcheck https://download1.rpmfusion.org/nonfree/el/rpmfusion-nonfree-release-8.noarch.rpm
sudo dnf -y install ffmpeg
sudo dnf -y install ffmpeg-devel

### make scripts executable
sudo chmod u+x video_prep/*.sh
sudo chmod u+x video_prep/*.py

### install docker
./video_prep/install_docker.sh

### build docker
docker build video_prep --tag bag2video

subject_padded='005'
### download files
mkdir -p ./data
oci os object bulk-download -bn 'rrl-flo-raw' --dest-dir ./data --prefix "$subject_padded"

### uncompress files
find ./data -name '*.bz2' -exec lbzip2 -d -f -n 48 {} \;

### parallel 1 (if podium)
if [ -d "./data/$subject_padded/ros/podium/" ]
then
./video_prep/run_docker_bag2vid.sh -d "$(pwd)/data/$subject_padded/ros/podium" -s 90 -v info --audio_topic /robot_audio/audio_relay /lower_realsense/color/image_raw_relay /upper_realsense/color/image_raw_relay
./video_prep/transcode-to_davinci.sh -t "$(pwd)/data/$subject_padded/ros/podium"
fi

### parallel 2 (if robot)
if [ -d "./data/$subject_padded/ros/robot/" ]
then
./video_prep/run_docker_bag2vid.sh -d "$(pwd)/data/$subject_padded/ros/robot" -s 90 -v info --audio_topic /robot_audio/audio_relay /lower_realsense/color/image_raw_relay /upper_realsense/color/image_raw_relay
./video_prep/transcode-to_davinci.sh -t "$(pwd)/data/$subject_padded/ros/robot"
fi

### parallel 3 (if all together)
if ls "./data/$subject_padded/ros/"*.bag &> /dev/null
then
./video_prep/run_docker_bag2vid.sh -d "$(pwd)/data/$subject_padded/ros" -s 90 -v info --audio_topic /robot_audio/audio_relay /lower_realsense/color/image_raw_relay /upper_realsense/color/image_raw_relay
./video_prep/transcode-to_davinci.sh -t "$(pwd)/data/$subject_padded/ros"
fi

### parallel 4 (for gopro)
./video_prep/concatenate_vids.sh -t "$(pwd)/data/$subject_padded/gopro"
./video_prep/transcode-to_davinci.sh -t "$(pwd)/data/$subject_padded/gopro/concatenated"

### parallel 5 (for 3rd-person)
./video_prep/concatenate_vids.sh -t "$(pwd)/data/$subject_padded/3rd-person"
./video_prep/transcode-to_davinci.sh -t "$(pwd)/data/$subject_padded/3rd-person/concatenated"

### upload data
oci os object bulk-upload -bn 'rrl-flo-vids' --src-dir "./data/$subject_padded" --include '*.mov' --include '*.MOV' --include '*.mp4' --include '*.MP4' --include '*.avi' --include '*.AVI' --prefix "$subject_padded/"

rm -rf "data/$subject_padded"
