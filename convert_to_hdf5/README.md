# Convet To HDF5

This package converts bag files from the Lil'Flo platform to HDF5 files.

## Running

### With Docker

Temporary:

1.  Put a bag file named `in.bag` into a data folder somewhere
2.  Run `test.sh <path to data folder>` from within this directory
3.  Your HDF5 file will now be in in your data folder

### OCI

1.  push the code files to OCI by running `./oci_utilities/push_code.sh`
2.  Create an instance that does what you need using Oracle Linux 8 (on OCI web interface, see relevant SOP). Recomendations: AD2, Oracle Linux 8, VM.Standard2.24, private subnet, No SSH Keys, Boot Vol Size: 2TB
3.  Enable Bastion Service on the instance
4.  Remote into that instance. Ex:
    `oci-cli-helpers/utilities/oci-ssh.sh $(oci-cli-helpers/utilities/ocid.sh instance flo-hdf5-1)`
5.  Setup permissions: `OCI_CLI_AUTH=instance_principal && export OCI_CLI_AUTH`
6.  Install the oci cli: `sudo dnf -y install oraclelinux-developer-release-el8 && sudo dnf -y install python36-oci-cli`
7.  Pull down code onto the remote instance:
    `oci os object bulk-download -bn 'rrl-flo-run' --download-dir "$HOME/LilFloAssessmentPipeline" --overwrite`
8.  Run setup script: `chmod u+x "$HOME/LilFloAssessmentPipeline/oci_utilities/convert_to_hdf5/machine_setup.sh" && mkdir -p "$HOME/logs/install/" && bash "$HOME/LilFloAssessmentPipeline/oci_utilities/convert_to_hdf5/machine_setup.sh" 2>&1 | tee -a "$HOME/logs/install/$(date +"%Y-%m-%d-%H-%M-%S-%N" | cut -b1-22)"`
9.  Run tmux: `tmux`. If you disconnect, reconect: `tmux a`. You could also use screen.
10. Run Script: ` bash "$HOME/LilFloAssessmentPipeline/oci_utilities/convert_to_hdf5/run_manual.sh" <subj number> 2>&1 | tee -a "$HOME/logs/runs/$(date +"%Y-%m-%d-%H-%M-%S-%N" | cut -b1-22)-subj_<subj number>"  `

If you want to run a bunch of subjects at once, you can do that with something like:

```{bash}
for sn in 3
do
log="$HOME/logs/runs/$(date +"%Y-%m-%d-%H-%M-%S-%N" | cut -b1-22)-subj_$sn"
bash "$HOME/LilFloAssessmentPipeline/oci_utilities/convert_to_hdf5/run_manual.sh" "$sn" 2>&1 | tee -a $log
done
```

## TODO

1.  Explore using szip as a filter for compression
2.  Explore using smaller chunk sizes

### Note:

Please make sure there is a extrinsics.yaml file in the /data folder
