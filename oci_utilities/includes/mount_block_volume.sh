#!/usr/bin/env bash

# should be sourced by another file in which $cannonical_disk is defined
# will mount the filesystem at $cannonical_disk1 (ex: sdb1) on /mnt/subj-data
# Will set volume_mounted to true once mounting is setup.
# sets permissions so that filesystem can actually be used

# mount and setup permissions
echo "mounting drive"
# shellcheck disable=SC2154,SC2034
sudo mount "/dev/$cannonical_disk"1 /mnt/subj-data && volume_mounted=true
echo "setting drive permissions"
sudo chmod -R a+rwX /mnt/subj-data
