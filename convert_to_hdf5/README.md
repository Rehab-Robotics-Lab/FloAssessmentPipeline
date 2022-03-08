# Convet To HDF5

This package converts bag files from the Lil'Flo platform to HDF5 files.

## Running

### Local

1.  Put bag files in a directory (uncompressed)
2.  Install Docker
3.  Run `convert_to_hdf5/src/run_local -h` for instructions on running:
    1.  `convert_to_hdf5/src/run_local -r` will rebuild the relevant docker file
    2.  `convert_to_hdf5/src/run_local -r -d <path to dir with bag files>` will rebuild the relevant docker file and run the hdf5 conversion
    3.  `convert_to_hdf5/src/run_local -d <path to dir with bag files>` will run the hdf5 conversion

Once the conversion is complete, you should see the vid and novid hdf5 files in the directory
next to the bag files.

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
11. To keep an eye on the process running under the hood in docker, you can run:
    `while true; do sleep 1; docker logs -f hdf5-converter 2>&1 | tee -a "$HOME/logs/runs/docker"; done`
    in another tab (in tmux: `ctrl-b + "`)

If you want to run a bunch of subjects at once, you can do that with something like:

```{bash}
for sn in 3
do
log="$HOME/logs/runs/$(date +"%Y-%m-%d-%H-%M-%S-%N" | cut -b1-22)-subj_$sn"
bash "$HOME/LilFloAssessmentPipeline/oci_utilities/convert_to_hdf5/run_manual.sh" "$sn" 2>&1 | tee -a $log
done
```

All subjects

```{bash}
source ./LilFloAssessmentPipeline/oci_utilities/includes/get_all_subj.sh
for sn in $subjects
do
log="$HOME/logs/runs/$(date +"%Y-%m-%d-%H-%M-%S-%N" | cut -b1-22)-subj_$sn"
bash "$HOME/LilFloAssessmentPipeline/oci_utilities/convert_to_hdf5/run_manual.sh" "$sn" 2>&1 | tee -a $log
done
```
