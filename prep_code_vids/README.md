# Concatenate Videos

In order to do video coding, videos must be concatenated and renamed. In order to do this, you must:

1.  Get videos onto your machine (local or remote)
2.  Run the concatenation script
3.  Change the filename to be standardized
4.  Put the files onto Penn+Box

## Get videos onto your machine

You can take the standard method of downloading videos via the gui. This is not the best.
Alternatively you can create a password on Penn+Box (not your penn password) and then
use lftp to download files (Refer to main readme)

You could also use a tool like filezilla

## Run concatenation script

The concatenation script needs to be given a folder to look for gopro videos in. For example: `~/Documents/git/LilFloAssessmentPipeline/prep_code_vids/concatenate_vids.sh -t ~/Downloads/sdata/006/gopro`

Your concatenated files should now be in a subfolder of the gopro folder

## Composing in DaVinci Resolve:

For each subject:

1.  Concatenate gopro videos by running `concatenate_vids.sh` on the gopro folder
2.  Get video from ros bags by running `bag2video.py` on the ros folder ex: `./run_docker_bag2vid.sh -d ~/Downloads/003/ros -s 90 -v info --audio_topic /robot_audio/audio_relay /lower_realsense/color/image_raw_relay /upper_realsense/color/image_raw_relay`. Note: you may have to take ownership of the files: `chown $USER *.mp4`
3.  Transcode the ros files by running `transcode-to_davinci.sh -t <...>/ros`
4.  Transcode the gopro files by running `transcode-to_davinci.sh -t <...>/gopro/concatenated`
5.  Create a new project in DaVinci Reseolve, Set project as 3200x1440; 59.94 fps
6.  Create 3 timelines, one each for `in_person`, `classical`, `augmented`
7.  Import all of the transcoded videos into DaVinci Resolve
8.  Add in the appropriate videos for each timeline, the gopro x position is 640, the ros x position is -960
9.  Align the videos based on the video, mute the ros audio feed
10. Export with QuickTime DNxHR HQX 10-bit, custom resolution of 3200x1440, frame rate 59.94, quality best; Audio: Linear PCM, bit depth 24 or 16
11. Export the project (not archive, just project)
12. Transcode using the `transcode-to_maxQDA.sh` file
13. Upload to Box the transcoded video and the project export (drp file)

## Working with OCI

1.  push the necessary files to OCI by running `<>/oci_utilities/video_prep/upload.sh`
2.  Create an instance that does what you need using Oracle Linux 8 (on OCI web interface, see relevant SOP). Recomendations: AD2, Oracle Linux 8, VM.Standard2.24, private subnet, No SSH Keys, Boot Vol Size: 2TB
3.  Enable Bastion Service
4.  Remote into that instance. Ex:
    `./utilities/oci-ssh.sh $(./utilities/ocid.sh instance flo-vid-prep-ol8-intel-2tb_1)`
5.  Setup permissions: `OCI_CLI_AUTH=instance_principal && export OCI_CLI_AUTH`
6.  Pull down code: `oci os object bulk-download -bn 'rrl-flo-run' --download-dir '.' --overwrite`
7.  Add the extra repos: `./video_prep/add_extended_repos.sh`
8.  Install screen: `sudo dnf -y install screen`
9.  Run screen: `screen`. If you disconnect,  reconect: `screen -r`
10. Make scripts executable: `sudo chmod u+x video_prep/*.sh && sudo chmod u+x video_prep/*.py`
11. Install docker: `./video_prep/install_docker.sh`
12. Install lbzip2: `sudo dnf install -y lbzip2`
13. Install ffmpeg: `./video_prep/install_ffmpeg`
14. Build Docker Image: `docker build video_prep --tag bag2video`
15. Run Script: `./video_prep/run_manual.sh <subj number>`

If you want to run a bunch of subjects at once, you can do that with something like:

```{bash}
for sn in 1 3 4 5 9
do
./video_prep/run_manual.sh "$sn"
done
```

## Put files on Penn+Box

You can either drag and drop files back to Penn+Box or mount the FTP enpoint and copy them there.
