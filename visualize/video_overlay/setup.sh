#!/bin/bash
set -o errexit
set -o pipefail

sudo apt-get update -y
sudo apt-get upgrade -y
sudo apt-get install -y curl unzip wget

curl "https://awscli.amazonaws.com/awscli-exe-linux-aarch64.zip" -o "awscliv2.zip"
unzip awscliv2.zip
sudo ./aws/install
rm awscliv2.zip
rm -rf aws

curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh
sudo groupadd docker
sudo usermod -aG docker "$USER"
newgrp docker

sudo apt-get clean
