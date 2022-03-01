# Pose Detection

To do this, we run in a docker image, openpose is just easier that way.

We run openpose using the 25b model at non-maximum accuracy,
which requires 5.6 GB of GPU memory.
If multiple GPUs are available, they will all be used.

Note: there is a higher accuracy mode that takes 16GB of memory,
it just isn't worth it...

One of the steps here is finding the depth of the detected poses.
This requires that the camera intrinsics and depth-to-color extrinsics.
Sometimes this is avaialable in the HDF5 file directly, other times
it has to be read from the transforms.json file. That file is generated
using `get_transformss/run.sh`. This file is already generated and [stored
in OCI](https://cloud.oracle.com/object-storage/buckets/idtxkczoknc2/rrl-flo-transforms/objects?region=us-ashburn-1).

## Running

### On OCI

1.  push the code files to OCI by running `./oci_utilities/push_code.sh`
2.  Provision instance of:
    *   Image: Oracle Linux 8 GPU Build, [supported version](https://nvidia.github.io/nvidia-docker/)
    *   Shape: VM.GPU3.2 (for higher resolution use a GPU4 instance)
    *   vcn: flo vcn
    *   subnet: private
    *   SSH Keys: None
    *   Boot Volume: 2000GB
3.  Modify the performance level of the boot volume to higher performance
4.  Enable Bastion Service on the instance
5.  Remote into that instance. Ex:
    `oci-cli-helpers/utilities/oci-ssh.sh $(oci-cli-helpers/utilities/ocid.sh instance flo-hdf5-1)`
6.  Check if the nvidia drivers are working: `nvidia-smi`
7.  Setup permissions: `OCI_CLI_AUTH=instance_principal && export OCI_CLI_AUTH`
8.  Install the oci cli: `sudo dnf -y install oraclelinux-developer-release-el8 && sudo dnf -y install python36-oci-cli`
9.  Pull down code onto the remote instance:
    `oci os object bulk-download -bn 'rrl-flo-run' --download-dir "$HOME/LilFloAssessmentPipeline" --overwrite`
10. Run setup script: `chmod u+x "$HOME/LilFloAssessmentPipeline/oci_utilities/openpose/machine_setup.sh" && mkdir -p "$HOME/logs/install/" && bash "$HOME/LilFloAssessmentPipeline/oci_utilities/openpose/machine_setup.sh" 2>&1 | tee -a "$HOME/logs/install/$(date +"%Y-%m-%d-%H-%M-%S-%N" | cut -b1-22)"`
11. Test that nvidia docker installed properly:
12. Run tmux: `tmux`. If you disconnect, reconect: `tmux a`. You could also use screen.
13. Run Script: `bash "$HOME/LilFloAssessmentPipeline/oci_utilities/openpose/run_manual.sh" <subj number> 2>&1 | tee -a "$HOME/logs/runs/$(date +"%Y-%m-%d-%H-%M-%S-%N" | cut -b1-22)-subj_<subj number>"`

If you want to run a bunch of subjects at once, you can do that with something like:

```{bash}
for sn in 35
do
log="$HOME/logs/runs/$(date +"%Y-%m-%d-%H-%M-%S-%N" | cut -b1-22)-subj_$sn"
bash "$HOME/LilFloAssessmentPipeline/oci_utilities/openpose/run_manual.sh" "$sn" 2>&1 | tee -a $log
done
```

### Running Locally

1.  Install Dependencies:
    a. You can try to use the local setup script `pose-body/setup_local.sh`
    b. You can manually install: cuda, docker, nvidia-docker
2.  Run script: `./pose-body/scripts/run_local.sh -d <"Location to data directory">`

You might get a common GPU architecture not supported error. In that case, make sure to restart docker with :

`sudo systemctl restart docker`

Make sure you are able to run:
`sudo docker run --rm --gpus all nvidia/cuda:11.0-base nvidia-smi`

## General Architecture:

job def: subj no, hdf5 file name
parallelization: multiple hdf5 files (each subject has multiple)

Input: exiting HDF5 file with videos

There is a single output: an HDF5 file with the poses extracted from each view, the depth values (for depth cameras) at those pixel locations

1.  Look in HDF5 file for any datasets starting with `vid_color_data_` and run openpose on those
2.  if there is matching `vid_depth_data_` set, then extract the depth values that correspond to the pixel locations
3.  put in new hdf5 file with `pose_<upper/lower>` (need to think through that data structure a bit more I think)
