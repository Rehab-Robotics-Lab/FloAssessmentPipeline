#!/bin/bash
set -o errexit
set -o pipefail

script=$(realpath "$0")
scriptpath=$(dirname "$script")

echo "Running: $script from $scriptpath"

# Setup Virtual Machine for Video Prep Code

echo 'expand filesystem'
sudo /usr/libexec/oci-growfs -y || true

echo 'install oci-cli'
sudo dnf -y install oraclelinux-developer-release-el8
sudo dnf -y install python36-oci-cli

echo 'set permissions'
grep -qxF 'OCI_CLI_AUTH=instance_principal' ~/.bashrc || echo 'OCI_CLI_AUTH=instance_principal' >> ~/.bashrc
grep -qxF 'export OCI_CLI_AUTH' ~/.bashrc || echo 'export OCI_CLI_AUTH' >> ~/.bashrc

echo 'pull in scripts'
#oci os object bulk-download -bn 'rrl-flo-run' --download-dir '.' --overwrite

echo 'add in secondary repos'
# shellcheck source=../includes/add_extended_repos.sh
source "$scriptpath/../includes/add_extended_repos.sh"

echo 'install tmux'
sudo dnf install -y tmux

echo 'install lbzip2'
sudo dnf install -y lbzip2

echo 'install screen'
sudo dnf -y install screen

echo 'install ffmpeg'
sudo dnf -y install https://download.fedoraproject.org/pub/epel/epel-release-latest-8.noarch.rpm
sudo yum-config-manager --enable ol8_codeready_builder
sudo dnf -y localinstall --nogpgcheck https://download1.rpmfusion.org/free/el/rpmfusion-free-release-8.noarch.rpm
sudo dnf -y install --nogpgcheck https://download1.rpmfusion.org/nonfree/el/rpmfusion-nonfree-release-8.noarch.rpm
sudo dnf -y install ffmpeg
sudo dnf -y install ffmpeg-devel

echo 'make scripts executable'
sudo chmod u+x "$scriptpath/"*.sh
sudo chmod u+x "$scriptpath/../../prep_code_vids/"*.sh

echo 'install docker'
# shellcheck source=../includes/install_docker.sh
. "$scriptpath/../includes/install_docker.sh"

echo 'done installing docker'

echo 'build docker'
docker build -t bag2video -f "$scriptpath/../../dockerfiles/bag2video" "$scriptpath/../../"

echo 'create log folder for runs'
mkdir -p "$HOME/logs/runs/"

echo 'done with setup!!'
