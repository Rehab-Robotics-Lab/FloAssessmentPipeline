
sudo mdadm --create /dev/md0 --raid-devices=4 --level 0 /dev/nvme0n1 /dev/nvme1n1 /dev/nvme2n1 /dev/nvme3n1

sudo mkdir -p /mnt/data

sudo mkfs.ext4 /dev/md0

sudo mount /dev/md0 /mnt/data/

sudo chown opc /mnt/data/
