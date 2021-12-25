#!/bin/bash
set -o errexit
set -o pipefail

script=$(realpath "$0")
scriptpath=$(dirname "$script")

echo "Running: $script from $scriptpath"

# Setup Virtual Machine for convert to hdf5

echo 'expand filesystem'
sudo /usr/libexec/oci-growfs -y || true

echo 'install oci-cli'
sudo dnf -y install oraclelinux-developer-release-el8
sudo dnf -y install python36-oci-cli

echo 'set permissions'
grep -qxF 'OCI_CLI_AUTH=instance_principal' ~/.bashrc || echo 'OCI_CLI_AUTH=instance_principal' >> ~/.bashrc
grep -qxF 'export OCI_CLI_AUTH' ~/.bashrc || echo 'export OCI_CLI_AUTH' >> ~/.bashrc

echo 'add in secondary repos'
# shellcheck source=../includes/add_extended_repos.sh
source "$scriptpath/../includes/add_extended_repos.sh"

echo 'install tmux'
sudo dnf install -y tmux

echo 'install screen'
sudo dnf -y install screen

echo 'install jq'
sudo dnf -y install jq

echo 'make scripts executable'
sudo chmod u+x "$scriptpath/"*.sh
sudo chmod u+x "$scriptpath/../../openpose/"*.sh
sudo chmod u+x "$scriptpath/../../openpose/scripts/"*.sh
sudo chmod u+x "$scriptpath/../../openpose/scripts/"*.py

echo 'install docker'
# shellcheck source=../includes/install_docker.sh
source "$scriptpath/../includes/install_docker.sh"

echo 'install nvidia docker'

distribution=$(. /etc/os-release;echo $VERSION_ID)
curl -s -L "https://nvidia.github.io/nvidia-docker/rhel$distribution/nvidia-docker.repo" | \
  sudo tee /etc/yum.repos.d/nvidia-docker.repo

#distribution=$(. /etc/os-release;echo "$VERSION_ID") \
#   && curl -s -L "https://nvidia.github.io/nvidia-docker/rhel$distribution/nvidia-docker.list" | sudo tee /etc/yum.repos.d/nvidia-docker.repo

sudo dnf install -y nvidia-docker2

if [ "$(jq '."default-runtime"' /etc/docker/daemon.json -r)" == 'nvidia' ]
then
    echo 'docker daemon default runtime already set to nvidia'
else
    sudo cp /etc/docker/daemon.json /etc/docker/daemon.json.old
    jq '. + {"default-runtime": "nvidia"}' /etc/docker/daemon.json.old | sudo tee /etc/docker/daemon.json
    echo 'changed default docker daemon runtime to nvidia'
fi

sudo systemctl restart docker

echo 'done installing docker'

echo 'build docker'
docker build -t openpose -f "$scriptpath/../../dockerfiles/openpose" "$scriptpath/../../"

echo 'create log folder for runs'
mkdir -p "$HOME/logs/runs/"

echo 'done with setup!!'
