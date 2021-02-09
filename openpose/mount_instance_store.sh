#!/bin/bash
set -o errexit
set -o pipefail

sudo mkfs -t xfs -f /dev/nvme1n1
sudo mkdir -p /data
sudo mount /dev/nvme1n1 /data
sudo chown -R ubuntu /data
