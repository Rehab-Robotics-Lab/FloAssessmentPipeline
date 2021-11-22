#!/usr/bin/env bash

# Designed to be sourced from another script
# Attaches to a block volume, with disconnect on script end
# will set a variable $volume_mounted that goes true once mounted
# Requires $instance_ocid and $volume_id to be defined

# WARNING!!! If something else sets the trap for EXIT, uh oh..

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

cannonical_disk="$(sudo iscsiadm -m session -P 3 | grep "$volume_iqn\|disk" | grep "$volume_iqn" -A1 | tail -1 | cut -f4 -d ' ' | cut -f1)"
echo "found block storage at: /dev/$cannonical_disk"

echo "creating mount point"
sudo mkdir -p /mnt/subj-data
