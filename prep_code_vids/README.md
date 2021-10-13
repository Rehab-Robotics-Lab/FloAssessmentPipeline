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
11. Transcode using the `transcode-to_maxQDA.sh` file
12. Upload to Box

## Put files on Penn+Box

You can either drag and drop files back to Penn+Box or mount the FTP enpoint and copy them there.
