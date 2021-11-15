#!/usr/bin/env bash
set -o errexit
set -o pipefail

script="$(realpath "$0")"
scriptpath="$(dirname "$script")"

# shellcheck source=../includes/check_version.sh
source "$scriptpath/check_version.sh"

# shellcheck source=../includes/add_extended_repos.sh
source "$scriptpath/add_extended_repos.sh"

echo "installing lbzip2"
sudo dnf install -y -q lbzip2

# shellcheck source=../includes/set_permissions.sh
source "$scriptpath/set_permissions.sh"

# shellcheck source=../includes/parse_input_subj_no.sh
source "$scriptpath/parse_input_subj_no.sh"

# shellcheck source=../includes/get_instance_info.sh
source "$scriptpath/get_instance_info.sh"

# check if the relevant block storage vol exists, if not create it
echo "checking if block storage volume exists"
volume_id=$(
    oci bv volume list \
        --compartment-id "$compartment" \
        --display-name "flo_data/$subject_padded" \
        --query 'data[? "lifecycle-state" == `"AVAILABLE"`] | [0].id' \
        --raw-output
)
echo "block storage volume ocid: $volume_id"

if [ -z "$volume_id" ]
then
    echo "no volume found, creating new volume"
    volume_id=$(oci bv volume create \
        --availability-domain "$availability_domain" \
        --compartment-id "$compartment" \
        --display-name "flo_data/$subject_padded" \
        --is-auto-tune-enabled true \
        --size-in-gbs 7000 \
        --vpus-per-gb 20 \
        --wait-for-state AVAILABLE \
        --query data.id \
        --raw-output)
    echo "new volume ocid: $volume_id"
fi

# shellcheck source=../includes/attach_block_volume.sh
source "$scriptpath/attach_block_volume.sh"

# if not formatted make a partition and format
echo "checking if formatted"

#if [ "$(sudo sfdisk -d '/dev/oracleoci/oraclevdaa' 2>&1)" == "" ]
if [ "$(lsblk -f | grep "$cannonical_disk"1)" == "" ]
then
    echo "not formatted"
    echo "creating new partitiion"
    sudo parted --script "/dev/$cannonical_disk" \
        'mklabel gpt' \
        'mkpart primary ext4 1 -1'
    echo "creating new file system"
    sudo mkfs -F -t ext4 "/dev/$cannonical_disk"1
fi

# shellcheck source=../includes/mount_block_volume.sh
source "$scriptpath/mount_block_volume.sh"

# copy and decompress data
echo "creating raw data folder"
mkdir -p '/mnt/subj-data/raw'
echo "downloading files form object storage"
oci os object bulk-download \
    -bn 'rrl-flo-raw' \
    --download-dir '/mnt/subj-data/raw' \
    --prefix "$subject_padded" \
    --parallel-operations-count 1000 \
    --overwrite

echo "removing subject number prefix from directories"
#rsync -a "/mnt/subj-data/raw/$subject_padded/"* /mnt/subj-data/raw
cp -rlf "/mnt/subj-data/raw/$subject_padded/"* /mnt/subj-data/raw
rm "/mnt/subj-data/raw/$subject_padded" -r

echo "uncompressing files"
find /mnt/subj-data -name '*.bz2' -exec lbzip2 -d -n 48 {} \;
echo "untarring files"
find /mnt/subj-data -name '*.tar' -exec bash -c 'tar -xf "$1" --directory "$(dirname "$1")"' shell {} \;


echo "job complete"
