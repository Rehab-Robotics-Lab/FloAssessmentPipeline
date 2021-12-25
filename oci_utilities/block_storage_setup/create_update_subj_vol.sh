#!/usr/bin/env bash
set -o errexit
set -o pipefail

script="$(realpath "$0")"
scriptpath="$(dirname "$script")"

# shellcheck source=../includes/parse_input_subj_no.sh
source "$scriptpath/../includes/parse_input_subj_no.sh"

# yields: instance_ocid, availability_domain, instance_compartment_id, compartment
# shellcheck source=../includes/get_instance_info.sh
source "$scriptpath/../includes/get_instance_info.sh"

echo "running for subject: $subject_padded"
# check if the relevant block storage vol exists, if not create it
echo "checking if block storage volume exists"
# shellcheck disable=SC2154,SC2016
# Dont want to have shell expend this stuff (SC2016)
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

    size=$(oci os object list \
        -bn rrl-flo-hdf5 \
        --prefix '035' \
        --all \
        --query 'sum(data[].size)' \
        --raw-output)
    target_size=$(echo "($size*1.2*0.000000001)/1+1" | bc)
    echo "making new volume of size: $target_size GB"

    volume_id=$(oci bv volume create \
        --availability-domain "$availability_domain" \
        --compartment-id "$compartment" \
        --display-name "flo_data/$subject_padded" \
        --is-auto-tune-enabled true \
        --size-in-gbs "$target_size" \
        --vpus-per-gb 40 \
        --wait-for-state AVAILABLE \
        --query data.id \
        --raw-output)
    echo "new volume ocid: $volume_id"
fi

volume_mounted=false

disconnect_volume(){
    # detach block volume
    echo '------------------------'
    echo "disconnecting from volume in OS"
    $volume_mounted && \
        (sudo umount /mnt/subj-data && \
        echo "unmounted drive")

    echo "removing block storage attachment in OCI"
    [ -z "$attachment_id" ] || \
        (oci compute volume-attachment detach \
            --volume-attachment-id "$attachment_id"\
            --force\
            --wait-for-state DETACHED && \
            echo "removed block storage volume attachment")
}
trap disconnect_volume EXIT

# attach to the block volume
echo "attaching to block storage"
# shellcheck disable=SC2154
attachment_id=$(oci compute volume-attachment attach-iscsi-volume \
    --instance-id "$instance_ocid" \
    --volume-id "$volume_id" \
    --is-read-only false \
    --is-shareable false \
    --wait-for-state ATTACHED \
    --use-chap true \
    --query data.id \
    --device "/dev/oracleoci/oraclevdaa" \
    --raw-output)
echo "attachment ocid: $attachment_id"

echo "getting attachment details"
volume_device=$(oci compute volume-attachment get \
    --volume-attachment-id "$attachment_id" \
    --query 'data.device' \
    --raw-output
)
echo "block volume can be found at: $volume_device"

# if not formatted make a partition and format
echo "checking if formatted"

#if [ "$(sudo sfdisk -d '/dev/oracleoci/oraclevdaa' 2>&1)" == "" ]
if [ "$(lsblk -f | grep "$volume_device"1)" == "" ]
then
    echo "not formatted"
    echo "creating new partitiion"
    echo ';' | sudo sfdisk --force /dev/oracleoci/oraclevdaa
    sudo partprobe
    echo "creating new file system"
    sudo mkfs -F -t ext4 "$volume_device"1
fi

echo "creating mount point"
sudo mkdir -p /mnt/subj-data

# mount and setup permissions
echo "mounting drive"
# shellcheck disable=SC2154,SC2034
sudo mount "/dev/$cannonical_disk"1 /mnt/subj-data && volume_mounted=true
echo "setting drive permissions"
sudo chmod -R a+rwX /mnt/subj-data

# copy and decompress data
echo "creating raw data folder"
echo "downloading files from object storage"
oci os object bulk-download \
    -bn 'rrl-flo-hdf5' \
    --download-dir '/mnt/subj-data/raw' \
    --prefix "$subject_padded" \
    --parallel-operations-count 1000 \
    --overwrite

echo "removing subject number prefix from directories"
#rsync -a "/mnt/subj-data/raw/$subject_padded/"* /mnt/subj-data/raw
cp -rlf "/mnt/subj-data/$subject_padded/"* /mnt/subj-data/raw
rm "/mnt/subj-data/raw/$subject_padded" -r

disconnect_volume

echo "job complete"
