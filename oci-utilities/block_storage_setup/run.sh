#!/usr/bin/env bash
set -o errexit
set -o pipefail

echo "checking version"
echo "version: $(cat /etc/oracle-release)"

echo "installing extended yum repos"
sudo dnf install -y -q oracle-epel-release-el8
echo "enabling extended yum repos"
sudo dnf config-manager -q --set-enabled ol8_developer_EPEL
echo "installing lbzip2"
sudo dnf install -y -q lbzip2

# set to use principal auth using dynamic group permissions
echo "setting permissions"
OCI_CLI_AUTH=instance_principal
export OCI_CLI_AUTH

# pass in the subject number
echo "parsing input"
echo "Passed: $1"
re='^[0-9]+$'
if ! [[ $1 =~ $re ]] ; then
   echo "error: Subject number is not a number" >&2; exit 1
fi
subject_padded=$(printf '%03d' "$1")
echo "Processing for subj: $subject_padded"

# Get some info about where we are running
echo "getting run instance information"
#instance_ocid=$(oci-metadata --get id --value-only)
instance_ocid=$(curl -s -L http://169.254.169.254/opc/v1/instance/ | jq '.id' -r)

echo "instance ocid: $instance_ocid"
availability_domain=$(
    oci compute instance get \
        --instance-id "$instance_ocid" \
        --query 'data."availability-domain"'\
        --raw-output
    )
echo "availability domain: $availability_domain"


# check if the relevant block storage vol exists, if not create it
echo "checking if block storage volume exists"
volume_id=$(
    oci bv volume list \
        --compartment-id "$PROJECT_COMPARTMENT_OCID" \
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
        --compartment-id "$PROJECT_COMPARTMENT_OCID" \
        --display-name "flo_data/$subject_padded" \
        --is-auto-tune-enabled true \
        --size-in-gbs 7000 \
        --vpus-per-gb 20 \
        --wait-for-state AVAILABLE \
        --query data.id \
        --raw-output)
    echo "new volume ocid: $volume_id"
fi

# Safely exit if something goes wrong
volume_mounted=false

disconnect_volume(){
    # detach block volume
    echo '------------------------'
    echo "disconnecting from volume in OS"
    $volume_mounted && \
        (sudo umount /mnt/subj-data && \
        echo "unmounted drive")
    [ -z "$volume_iqn" ] || [ -z "$volume_ip" ] || [ -z "$volume_port" ] || \
        (sudo iscsiadm -m node -T "$volume_iqn" -p "$volume_ip:$volume_port" -u && \
        sudo iscsiadm -m node -o delete -T "$volume_iqn" -p "$volume_ip:$volume_port" && \
        echo "disconnected iscsi")

    echo "removing block storage attachment in OCI"
    [ -z "$attachment_id" ] || \
        (oci compute volume-attachment detach \
            --volume-attachment-id "$attachment_id"\
            --force\
            --wait-for-state DETACHED && \
            echo "removed block storage volume attachment")
}
trap disconnect_volume ERR EXIT

# attach to the block volume
echo "attaching to block storage"
attachment_id=$(oci compute volume-attachment attach-iscsi-volume \
    --instance-id "$instance_ocid" \
    --volume-id "$volume_id" \
    --is-read-only false \
    --is-shareable false \
    --wait-for-state ATTACHED \
    --use-chap true \
    --query data.id \
    --raw-output)
echo "attachment ocid: $attachment_id"

echo "getting attachment details"
volume_iqn=$(oci compute volume-attachment get \
    --volume-attachment-id "$attachment_id" \
    --query 'data.iqn' \
    --raw-output
)
volume_ip=$(oci compute volume-attachment get \
    --volume-attachment-id "$attachment_id" \
    --query 'data.ipv4' \
    --raw-output
)
volume_port=$(oci compute volume-attachment get \
    --volume-attachment-id "$attachment_id" \
    --query 'data.port' \
    --raw-output
)
volume_chap_username=$(oci compute volume-attachment get \
    --volume-attachment-id "$attachment_id" \
    --query 'data."chap-username"' \
    --raw-output
)
volume_chap_secret=$(oci compute volume-attachment get \
    --volume-attachment-id "$attachment_id" \
    --query 'data."chap-secret"' \
    --raw-output
)

echo "initiation iscsi attachment"
sudo iscsiadm -m node -o new -T "$volume_iqn" -p "$volume_ip:$volume_port"
sudo iscsiadm -m node -o update -T "$volume_iqn" -n node.startup -v automatic
sudo iscsiadm -m node -T "$volume_iqn" -p "$volume_ip:$volume_port" -o update -n node.session.auth.authmethod -v CHAP
sudo iscsiadm -m node -T "$volume_iqn" -p "$volume_ip:$volume_port" -o update -n node.session.auth.username -v "$volume_chap_username"
sudo iscsiadm -m node -T "$volume_iqn" -p "$volume_ip:$volume_port" -o update -n node.session.auth.password -v "$volume_chap_secret"
sudo iscsiadm -m node -T "$volume_iqn" -p "$volume_ip:$volume_port" -l


#cannonical_disk=$(readlink /dev/oracleoci/oraclevdaa -f)
# TODO: there has to be a better way to do this
sleep 5
echo "iscsiadmn disks:"
sudo iscsiadm -m session -P 3 | grep 'Target\|disk'

cannonical_disk="/dev/$(sudo iscsiadm -m session -P 3 | grep "$volume_iqn\|disk" | grep "$volume_iqn" -A1 | tail -1 | cut -f4 -d ' ' | cut -f1)"
echo "found block storage at: $cannonical_disk"

echo "creating mount point"
sudo mkdir -p /mnt/subj-data

# if not formatted make a partition and format
echo "checking if formatted"
#if [ "$(sudo sfdisk -d '/dev/oracleoci/oraclevdaa' 2>&1)" == "" ]
if [ "$(lsblk -f | grep "$cannonical_disk"1)" == "" ]
then
    echo "not formatted"
    echo "creating new partitiion"
    sudo parted --script "$cannonical_disk" \
        'mklabel gpt' \
        'mkpart primary ext4 1 -1'
    echo "creating new file system"
    sudo mkfs -F -t ext4 "$cannonical_disk"1
fi

# mount and setup permissions
echo "mounting drive"
sudo mount "$cannonical_disk"1 /mnt/subj-data && volume_mounted=true
echo "setting drive permissions"
sudo chmod -R a+rwX /mnt/subj-data

# copy and decompress data
echo "creating raw data folder"
mkdir -p '/mnt/subj-data/raw'
echo "downloading files form object storage"
oci os object bulk-download \
    -bn 'rrl-flo-raw' \
    --download-dir '/mnt/subj-data/raw' \
    --prefix "$subject_padded" \
    --parallel-operations-count 100

echo "removing subject number prefix from directories"
to_remove=$(ls /mnt/subj-data/raw)
mv /mnt/subj-data/raw/*/* /mnt/subj-data/raw
rm "/mnt/subj-data/raw/$to_remove" -r

echo "uncompressing files"
find /mnt/subj-data -name '*.bz2' -exec lbzip2 -d {} \;
echo "untarring files"
find /mnt/subj-data -name '*.tar' -exec tar -xvf {} \;


echo "job complete"
