echo "installing extended yum repos"
sudo dnf install -y -q oracle-epel-release-el8
echo "enabling extended yum repos"
sudo dnf config-manager -q --set-enabled ol8_developer_EPEL
