# mount and setup permissions
echo "mounting drive"
sudo mount "/dev/$cannonical_disk"1 /mnt/subj-data && volume_mounted=true
echo "setting drive permissions"
sudo chmod -R a+rwX /mnt/subj-data
