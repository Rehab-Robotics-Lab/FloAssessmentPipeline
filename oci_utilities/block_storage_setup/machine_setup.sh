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

echo 'create log folder for runs'
mkdir -p "$HOME/logs/runs/"

echo 'done with setup!!'
