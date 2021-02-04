#!/bin/bash
set -o errexit
set -o pipefail

sudo apt-get update -y && sudo apt-get upgrade -y
sudo apt-get install -y nvme-cli
#sudo apt-get install -y ubuntu-desktop
#sudo apt-get install -y gnome-panel gnome-settings-daemon metacity nautilus gnome-terminal
sudo apt-get install -y xfce4
sudo apt-get install -y xfce4-terminal
sudo apt-get install -y xrdp
sudo apt-get install -y htop
sudo apt-get install -y ffmpeg
sudo sed -i.bak '/fi/a #xrdp multiple users configuration \n xfce-session \n' /etc/xrdp/startwm.sh
echo xfce4-session > $HOME/.xsession
sudo adduser xrdp ssl-cert
chmod +x .xsession
sudo systemctl restart xrdp

curl "https://awscli.amazonaws.com/awscli-exe-linux-aarch64.zip" -o "awscliv2.zip"
unzip awscliv2.zip
sudo ./aws/install
rm awscliv2.zip
rm -rf aws

mkdir ~/.aws
cat <<EOT >> ~/.aws/config
[default]
credential_source = Ec2InstanceMetadata
region = us-east-1
output = table
EOT

sudo apt-get install -y vlc
sudo apt-get install -y lbzip2

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt-get update -y
sudo apt-get install -y ros-noetic-desktop
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

sudo apt-get install -y tightvncserver

sudo apt-get clean
