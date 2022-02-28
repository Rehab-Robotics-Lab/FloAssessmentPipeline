# Concatenate Videos

In order to do video coding, videos must be extracted from the bag files, concatenated, stitched,
and re-uploaded.

## Extract from Bag Files, Concatenate, Transcode

1.  push the code files to OCI by running `./oci_utilities/push_code.sh`
2.  Create an instance that does what you need using Oracle Linux 8 (on OCI web interface, see relevant SOP). Recomendations: AD2, Oracle Linux 8, VM.Standard2.24, private subnet, No SSH Keys, Boot Vol Size: 2TB
3.  Enable Bastion Service on the instance
4.  Remote into that instance. Ex:
    `oci-cli-helpers/utilities/oci-ssh.sh $(oci-cli-helpers/utilities/ocid.sh instance flo-vid-prep-ol8-intel-2tb_1)`
5.  Setup permissions: `OCI_CLI_AUTH=instance_principal && export OCI_CLI_AUTH`
6.  Install the oci cli: `sudo dnf -y install oraclelinux-developer-release-el8 && sudo dnf -y install python36-oci-cli`
7.  Pull down code onto the remote instance:
    `oci os object bulk-download -bn 'rrl-flo-run' --download-dir "$HOME/LilFloAssessmentPipeline" --overwrite`
8.  Run setup script: `chmod u+x "$HOME/LilFloAssessmentPipeline/oci_utilities/video_prep/machine_setup.sh" && mkdir -p "$HOME/logs/install/" && bash "$HOME/LilFloAssessmentPipeline/oci_utilities/video_prep/machine_setup.sh" >> "$HOME/logs/install/$(date +"%Y-%m-%d-%H-%M-%S-%N" | cut -b1-22)"`
9.  Run tmux: `tmux`. If you disconnect, reconect: `tmux a`. You could also use screen.
10. Run Script: `  bash "$HOME/LilFloAssessmentPipeline/oci_utilities/video_prep/run_manual.sh" <subj number> >> "$HOME/logs/runs/$(date +"%Y-%m-%d-%H-%M-%S-%N" | cut -b1-22)-subj_<subj number>" 2>&1 `

If you want to run a bunch of subjects at once, you can do that with something like:

```{bash}
for sn in 1 3 4 5 9
do
log="$HOME/logs/runs/$(date +"%Y-%m-%d-%H-%M-%S-%N" | cut -b1-22)-subj_$sn"
bash "$HOME/LilFloAssessmentPipeline/oci_utilities/video_prep/run_manual.sh" "$sn" >> $log 2>&1
done
```

If you want to run on all subjects, first run:
`source ./LilFloAssessmentPipeline/oci_utilities/includes/get_all_subj.sh`

which will create a variable `$subjects` which you can then use in your for loop.
If you want the reverse:
`$(echo $subjects  | awk '{ for (i=NF; i>1; i--) printf("%s ",$i); print $1; }')`

you might want to put the output into a log:
`./video_prep/run_manual.sh "$sn" >>> log.txt`

This will pull down the files for that subject, extract videos from the bag files,
concatenate the gopro videos, transcode all of the videos, and push them up to the
`rrl-flo-vids` bucket on OCI.

## Composing in DaVinci Resolve:

It is then necessary to pull down the videos to your local machine and stitch together a video
with the multiple feeds for video coding.

For each subject:

1.  Pull down the videos by running `oci_utilities/video_prep/pull_vids <subj number>`
2.  Create a new project in DaVinci Reseolve, Set project as 3200x1440; 59.94 fps
3.  Create 3 timelines, one each for `in_person`, `classical`, `augmented`
4.  Import all of the transcoded videos into DaVinci Resolve
5.  Add in the appropriate videos for each timeline, the gopro x position is 640, the ros x position is -960
6.  Align the videos based on the video, mute the ros audio feed. If the gopro video is not available, you can import the 3rd-person video, sync it up by video, hide the video, and use the audio.
7.  Export with QuickTime DNxHR HQX 10-bit, custom resolution of 3200x1440, frame rate 59.94, quality best; Audio: Linear PCM, bit depth 24 or 16
8.  Export the project (not archive, just project)
9.  Transcode using the `./prep_code_vids/transcode-to_maxQDA.sh -t <directory with resolve output>`
10. Upload to Box the transcoded video and the project export (drp file)

## Put files on Penn+Box

You can either drag and drop files back to Penn+Box or use FTP (see main repo readme)
