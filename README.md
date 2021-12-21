# FloAssessmentPipeline

This is the pipeline for assessing patient function based on data from the Flo System.
The fundamental idea is to operate over a 3 step pipeline:

1.  Extract pose
2.  Calculate features of motion
3.  Classify/Regress measure of function

Along the way there is data uploading, cleaning, manipulation, and visualization.

Data is ingested as compressed bag files. That data is put into HDF5 files.
All of the non-video generated data (poses, etc.) go into a separate hdf5 file
to make it easier to manage.

Everything can be run locally, but is really meant to run on Oracle Cloud Infrastructure (OCI).

## Some tools

- **ViTables:** is really great for being able to explore hdf5 files

## Organization of this repository

- Each step of the pipeline more or less gets its own directory.
- `common` directory for python code used across multiple other steps
- `dockerfiles` for dockerfiles to run the various tools
- `oci_utilities` scripts to help run the pipeline on OCI

## Getting Set with OCI

Start by [installing the OCI CLI](https://docs.oracle.com/en-us/iaas/Content/API/SDKDocs/cliinstall.htm).

There are a few different ways to authenticate yourself to work with
resources on OCI. The only secure one is to use tokens. To do this,
run [the authentication script](https://github.com/Rehab-Robotics-Lab/oci-cli-helpers/blob/main/auth/token-alive.sh)
and export the following variables into your shell (easiest to do this
from your bashrc/zshrc):

```{bash}
export OCI_CLI_PROFILE='token-oci-profile'
export OCI_CLI_CONFIG_FILE="$HOME/.oci/config"
export OCI_CLI_AUTH='security_token'
```

Any commands from the oci cli should now work.

## Pipeline

### Uploading data

1.  create a folder tree:
    - the subject number (three digits `NNN` ex: `009` or `024`)
      - gopro
      - 3rd-person
      - ros
        - robot
        - podium
2.  Reindex any incomplete bag files: `./upload_data/repair_bags.sh -t <directory with bag files>`
    and you may want to use the `-v` option to see progress
3.  packup the parameter files: `./upload_data/tar-meta.sh -t <directory with parameter files>`
4.  put the compressed bag files and tar parameter file into either the robot or podium directories
    based on where the files came from.
5.  Compress the bag files: `find <dir with bag files> -name '*.bag' -exec lbzip2 -v {} \;`
6.  put all of the gopro videos into the gopro folder
7.  put all of the 3rd person videos into the 3rd-person folder

#### Upload data to Penn+Box

There are two ways to upload to Penn+Box. Using either the web console or FTP.
The web console is easier to use but if the upload stops, you are up a creek, and it is kind of slow.
FTP requires you to know what you are doing a bit better, but can be faster and can handle upload
interuptions.

- Drag the subjects folder into the [Penn+Box Folder](https://upenn.app.box.com/folder/126576235920)

OR

- Setup a box password (this is seperate from you Penn password and only works on box)
- install lftp
- connect: `lftp -u <pennid>@upenn.edu ftp.box.com` then enter your box (not penn) password
- Now you can navigate around. For example, use cd to go to where you want to put files
  on box and lcd to where the files are on your "local" machine
- mirror the "local" machine to Box using `mirror --reverse --parallel=<num parallel uploads>`

#### Uploading to OCI

Uploading to OCI requires authentication which is granted through a login token.
You could do this manually using [the lab's oci tools](https://github.com/Rehab-Robotics-Lab/oci-cli-helpers/tree/main/auth)
Or you can directly use the upload script `./upload_data/upload_to_oci.sh -t <src dir to upload> -p <prefix>`.
The prefix should be the subject number, fully padded, ex: `009` or `020`.

### Generating video files

We need to be able to generate video files for reviewing studies, sharing work, and coding video.
To do this, refer to `prep_code_vids/README.md`

### Converting rosbags into hdf5

In order to make processing easier, we move everything into HDF5 files.
This allows easier indexing and out of order processing

For instructions on running, see: [convert_to_hdf5/README.md](convert_to_hdf5/README.md)

### Staging Data on Block Volume

To improve performance of the remaining steps, we will stage all of the
HDF5 files on block volumes. Each subject will get 1 block volume which
is sized to hold 1.2 times the size of the HDF5 files for that subject.
Each block volume will use auto-performance tiering bursting to 40 VPUs
when connected. 40 VPUs yields 1,080 MBPS -> 8.64 Gbps. So we will use
instances that give just a bit more than that. In order to use the UHP
pipe for the 40 VPU block volumes, the instance must have 16 cores.

1. Create a new instane:
   - VM.Standard.E4.Flex with 16 cores and 32 GB of RAM
   - Oracle Linux 8
   - Private VCN
   - No SSH Keys
   - Enable Bastion Service
   - Enable Block Volume Management
2. Push the code files to OCI by running `./oci_utilities/push_code.sh`
3. Remote into that instance. Ex:
   `oci-cli-helpers/utilities/oci-ssh.sh $(oci-cli-helpers/utilities/ocid.sh instance rrl-flo-blocksetup_0)`
4. Setup permissions: `OCI_CLI_AUTH=instance_principal && export OCI_CLI_AUTH`
5. Install the oci cli: `sudo dnf -y install oraclelinux-developer-release-el8 && sudo dnf -y install python36-oci-cli`
6. Pull down code onto the remote instance:
   `oci os object bulk-download -bn 'rrl-flo-run' --download-dir "$HOME/LilFloAssessmentPipeline" --overwrite`
7. Run setup script: `chmod u+x "$HOME/LilFloAssessmentPipeline/oci_utilities/block_storage_setup/machine_setup.sh" && mkdir -p "$HOME/logs/install/" && bash "$HOME/LilFloAssessmentPipeline/oci_utilities/block_storage_setup/machine_setup.sh" 2>&1 | tee -a "$HOME/logs/install/$(date +"%Y-%m-%d-%H-%M-%S-%N" | cut -b1-22)"`
8. Enter screen: `screen -R`
9. Run
   - single subject: `LilFloAssessmentPipeline/oci_utilities/block_storage_setup/create_update_subj_vol.sh <subj no>`
   - multi subject: `LilFloAssessmentPipeline/oci_utilities/block_storage_setup/create_update_all_subj_vol.sh`

### Backfill Camera Extrinsics

Some of the trials don't have all of the data they need for the extrinsics for the depth/
rgb. Luckily, this doesn't change too much over time. So we can grab data from other trials.
To do that, do this:

1. Create a new instane:
   - VM.Standard.E4.Flex with 16 cores and 32 GB of RAM
   - Oracle Linux 8
   - Private VCN
   - No SSH Keys
   - 3000 GB boot volume
   - Enable Bastion Service
   - Enable Block Volume Management
   - Edit boot volume to have 20 VPUs and auto tune
2. Push the code files to OCI by running `./oci_utilities/push_code.sh`
3. Remote into that instance. Ex:
   `oci-cli-helpers/utilities/oci-ssh.sh $(oci-cli-helpers/utilities/ocid.sh instance rrl-flo-blocksetup_0)`
4. Setup permissions: `OCI_CLI_AUTH=instance_principal && export OCI_CLI_AUTH`
5. Install the oci cli: `sudo dnf -y install oraclelinux-developer-release-el8 && sudo dnf -y install python36-oci-cli`
7. Pull down code onto the remote instance:
   `oci os object bulk-download -bn 'rrl-flo-run' --download-dir "$HOME/LilFloAssessmentPipeline" --overwrite`
6. Install python/pip: `sudo dnf -y install python3`
7. extract from:
```{python}
import h5py
f=h5py.File(file)
f
```

### Pose Detection

A central component of the pipeline is extracting pose from video.
There are a few different tools that can be used to do that, all of them
imperfect.

#### OpenPose

OpenPose provides 2D pose of subjects. You can view more information at [openpose/README.md](openpose/README.md).

## Bag File Review

View `inspect/README.md`

## Getting files from Penn+Box

During development, testing, etc. It might be good to be able to download data from Penn+Box. The best way to do this with with lftp:

1.  `sudo apt install lftp`
2.  `lftp <penn username>@upenn.edu@ftp.box.com`
3.  use `lcd` `!ls` `cd` and `ls` to navigate the remote and local directories
4.  Get files:
    - Use mirror to get an entire directory of files: `mirror --parallel=<num_parallel_files> <remote source> <local destination>`
    - Use pget to get a file via multiple parallel streams: `pget <filename>`
    - Use mget to get files. Something like `mget flo_recording_2020-12-16-15-3* -P 10` might be useful, 10 is saying to download up to 10 files simultaneously.
    - Note, you can put an `&` at the end of any command to be able to start the next command. You can recover a command with `wait <command number (shown when you put it in background)>`, you can send it back to the background with ctrl-z. You can view all jobs with `jobs`

## Notes

- I thought that using dense I/O shapes would make work faster. For reasons I don't understand, it does not.
- Block volumes aren't as fast as one might
