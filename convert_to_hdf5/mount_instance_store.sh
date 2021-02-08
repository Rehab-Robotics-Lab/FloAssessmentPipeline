#!/bin/bash
set -o errexit
set -o pipefail

sudo pvcreate /dev/nvme1n1 /dev/nvme2n1
sudo vgcreate VG_DATA /dev/nvme1n1 /dev/nvme2n1
sudo lvcreate -l 100%FREE -n DATA VG_DATA
sudo mkfs.ext3 /dev/VG_DATA/DATA
sudo mkdir -p /data
sudo mount /dev/VG_DATA/DATA /data
sudo chown -R ubuntu /data
