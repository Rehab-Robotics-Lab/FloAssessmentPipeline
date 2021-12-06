#!/usr/bin/env bash

#echo "reinstall systemd"
#sudo dnf reinstall systemd
echo "add dnf utils"
sudo dnf install -y dnf-utils zip unzip
echo "add docker repo"
sudo dnf config-manager --add-repo=https://download.docker.com/linux/centos/docker-ce.repo
#dnf remove -y runc
echo "install docker ce"
sudo dnf install -y docker-ce --nobest
echo "install systemd"
sudo dnf install -y systemd
echo "enable docker service"
sudo systemctl enable docker.service
echo "staart docker service"
sudo systemctl start docker.service
echo "add docker group"
sudo groupadd docker
echo "add docker user"
sudo usermod -aG docker "$USER"
echo "init docker group"
newgrp docker
# useradd docker_user
# echo "docker_user  ALL=(ALL)  NOPASSWD: /usr/bin/docker" >> /etc/sudoers
# echo "alias docker=\"sudo /usr/bin/docker\"" >> /home/docker_user/.bash_profile
# su - docker_use
