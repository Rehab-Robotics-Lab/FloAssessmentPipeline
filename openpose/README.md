# OpenPose for Pose Detection

To do this, we run in a docker image, openpose is just easier that way.

We run openpose at maximum accuracy. This requires a GPU with at least 16GB of memory.
If multiple GPUs are available, they will all be used.

## On OCI

1.  push the code files to OCI by running `./oci_utilities/push_code.sh`
1.  Provision instance of:
    - Image: Oracle Linux 8 GPU Build, [supported version](https://nvidia.github.io/nvidia-docker/)
    - Shape: VM.GPU3.2
    - vcn: flo vcn
    - subnet: private
    - SSH Keys: None
    - Boot Volume: 2000GB
1.  Enable Bastion Service on the instance
1.  Remote into that instance. Ex:
    `oci-cli-helpers/utilities/oci-ssh.sh $(oci-cli-helpers/utilities/ocid.sh instance flo-hdf5-1)`
1.  Check if the nvidia drivers are working: `nvidia-smi`
1.  Setup permissions: `OCI_CLI_AUTH=instance_principal && export OCI_CLI_AUTH`
1.  Install the oci cli: `sudo dnf -y install oraclelinux-developer-release-el8 && sudo dnf -y install python36-oci-cli`
1.  Pull down code onto the remote instance:
    `oci os object bulk-download -bn 'rrl-flo-run' --download-dir "$HOME/LilFloAssessmentPipeline" --overwrite`
1.  Run setup script: `chmod u+x "$HOME/LilFloAssessmentPipeline/oci_utilities/openpose/machine_setup.sh" && mkdir -p "$HOME/logs/install/" && bash "$HOME/LilFloAssessmentPipeline/oci_utilities/openpose/machine_setup.sh" 2>&1 | tee -a "$HOME/logs/install/$(date +"%Y-%m-%d-%H-%M-%S-%N" | cut -b1-22)"`
1.  Test that nvidia docker installed properly:
1.  Run screen: `screen`. If you disconnect, reconect: `screen -r`. You could also use tmux.
1.  Run Script: `bash "$HOME/LilFloAssessmentPipeline/oci_utilities/convert_to_hdf5/run_manual.sh" <subj number> 2>&1 | tee -a "$HOME/logs/runs/$(date +"%Y-%m-%d-%H-%M-%S-%N" | cut -b1-22)-subj_<subj number>"`

If you want to run a bunch of subjects at once, you can do that with something like:

```{bash}
for sn in 3
do
log="$HOME/logs/runs/$(date +"%Y-%m-%d-%H-%M-%S-%N" | cut -b1-22)-subj_$sn"
bash "$HOME/LilFloAssessmentPipeline/oci_utilities/convert_to_hdf5/run_manual.sh" "$sn" 2>&1 | tee -a $log
done
```

## on AWS

1.  provision instance with Ubuntu 20
2.  type g4dn.2xlarge 20GB of disk space
3.  Push over code, ex: `scp -rp -i ~/.aws/keys/flo-exp-aim1-openpose-key.pem ~/Documents/git/LilFloAssessmentPipeline/openpose ubuntu@10.128.253.74:/home/ubuntu`
4.  run setup script
5.  Edit `/etc/docker/daemon.json` to have `"default-runtime": "nvidia",` as the second line, [like this](https://docs.nvidia.com/dgx/nvidia-container-runtime-upgrade/index.html#using-nv-container-runtime)
6.  build dockerfile: `docker build . --tag openpose`
7.  attach ssd: `./mount_instance_store.sh`
8.  Run script: `./scripts/process-in-ec2.sh -s <subj> -c <cond: augmented-telepresence, classical-telepresence, in-person> -a <activity: simon-says, target-touch>`

### Running Locally

1. Run script: ./setup_local.sh
2. Build openpose using: build dockerfile: `docker build . --tag openpose`
3. Run script: ./scripts/run_local.sh -d <"Location to data directory">

You might get a common GPU architecture not supported error. In that case, make sure to restart docker with :

sudo systemctl restart docker

Make sure you are able to run:
sudo docker run --rm --gpus all nvidia/cuda:11.0-base nvidia-smi

## General Architecture:

job def: subj no, hdf5 file name
parallelization: multiple hdf5 files (each subject has multiple)

Input: exiting HDF5 file with videos

There is a single output: an HDF5 file with the poses extracted from each view, the depth values (for depth cameras) at those pixel locations

1.  Look in HDF5 file for any datasets starting with `vid_color_data_` and run openpose on those
2.  if there is matching `vid_depth_data_` set, then extract the depth values that correspond to the pixel locations
3.  put in new hdf5 file with `pose_<upper/lower>` (need to think through that data structure a bit more I think)
