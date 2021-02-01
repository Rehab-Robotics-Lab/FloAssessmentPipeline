# FloAssessmentPipeline

This is the pipeline for assessing patient function based on data from the FloSystem

## Setting up:

- [install docker](https://docs.docker.com/get-docker/)
  - be sure to follow the post-install steps (https://docs.docker.com/engine/install/linux-postinstall/)
- [install pipenv](https://pipenv-fork.readthedocs.io/en/latest/index.html)
- install python dependencies with pipenv: `pipenv install`
- [install cuda drivers](https://docs.nvidia.com/cuda/cuda-quick-start-guide/index.html#linux)
                        ( https://developer.nvidia.com/cuda-downloads)
- [install nvidia-container-toolkit](https://github.com/NVIDIA/nvidia-docker#ubuntu-160418042004-debian-jessiestretchbuster)


## Running:

- `mkdir data` (The data folder should be at the root of the LilFloAssessment Folder)
- Put bag files in data directory
- `pipenv run python top_level_runner.py`

## Architecture

The idea is to have modules which represent a single type of processing
on the bag file. Processing is done by reading from the bag file and writing
the results back into it.

NOTE: Always backup your bag files in advance.

The modules are called in sequence by `top_level_runner.py`.

### Adding new modules

The system is designed to make it easy to add new modules.

1. Create a new new ros package in its own folder under the root.
   Be sure that your new ros package package file is up to date
2. Create a dockerfile to run your new module
3. Create a new python file with a class that extends the `ProcessingStep` class from the
   top_level_runner
4. Create a new `__init__.py` file to expose your new class
5. Import your new class into the `top_level_runner` and add
   it to the OPERATIONS array.

## Troubleshooting

The first thing to do is to see if you are hooking up to your
gpu well. Run: `pipenv run python test_gpu.py`

## TODO:

- [ ] Add more modules
- [ ] Currently using a PR branch for docker, when merged need to switch

## skething pipeline

The pipeline will be running in AWS batch

### getting rosbags into hdf5
job def: subj no
parallelization: multiple subjects at once

1. copy meta file for this subj
3. for each file in ros archive, in parallel (implemented in `convert_to_hdf5/src/run_script.bash`):
    - copy first bag file from s3 ros folder into local
    - uncompress: `lbzip2 -d filename` or other similar
3. for each rosbag, append the rosbag to the desired hdf5 file
    - use the mapping in the metadata file to determine which topics to record
    - scan through the hdf5 file to see if each message is within the time range for any of the segments from the meta file (note, this does not include calibration for now)
    - add any found messages on target topics to the hdf5 file for that experiment/activity/modality. Name hdf5 files per the segment names in the meta files ex: `in-person_simon-says_ros.hdf5`
    - use szip compression on the hdf5 file as a filter
4. put hdf5 file into the hdf5 s3 bucket with prefix for the appropriate subject: `flo-exp-aim1-data-hdf5/<user id, three digits: 008>`

### Getting gopro into hdf5
job def: subj no
parallelization: multiple subjects at once

Why are we putting go pro videos into hdf5 files? So that the rest of the pipeline can work on them without needing adjustment and to allow arbitrary frame access. 

Why don't we combine with the ros data? MJS proposed an approach where the grid would be tracked through the calibrationi process for both the gopro and the realsense sensors. The path could then be aligned in time and space giving both camera extrinsics as well as time offsets. Unfortunately, to propogate across go pro files, this would require an accurate offset between those files. The hope was that the date of creation could be used. Unfortunately gopros only store the creation date with second precision, so the alignment would be off by up to one second. With that level of misalignment, we might as well just keep them seperate.

1. copy meta file for this subj
2. copy the files needed from the calibration section
3. Develop an intrinsic calibration matrix from these files (maybe randomly select frames, check for a grid, get 300 frames with grid, then calibrate off those? could make better by selecting with sharp focus on grid and by looking for grids that are well spaced out, but start with random for now)
4. for each sequence, grab all of the frames, rectify them, and put them in their respective hdf5 file under the name `vid_gopro` name each hdf5 file per: `in-person_simon-says_gopro.hdf5`
4. put hdf5 file into the hdf5 s3 bucket with prefix for the appropriate subject: `flo-exp-aim1-data-hdf5/<user id, three digits: 008>`


### OpenPose
job def: subj no, hdf5 file name
parallelization: multiple hdf5 files (each subject has multiple)

Input: exiting HDF5 file with videos

There is a single output: an HDF5 file with the poses extracted from each view, the depth values (for depth cameras) at those pixel locations

1. Look in HDF5 file for any datasets starting with `vid_color_data_` and run openpose on those
2. if there is matching `vid_depth_data_` set, then extract the depth values that correspond to the pixel locations
3. put in new hdf5 file with `pose_<upper/lower>` (need to think through that data structure a bit more I think)

### QC and publications

we need a way to generate videos overlaying everything and visualizing it all to make sure it is workin and to put in publications

## Metadata
In order to run, we will need some metadata.
This is done in meta.yaml, which should be in each subjects root dir. 
This specifies the start and end times for each segment of the experiment. 

This is an example:
'''yaml
---
id: 008
segments:
  in-person:
    simon-says:
      ros:
        start: 1608148710.2
        end: 1608148919.8
      gopro:
        start: <frame number in first file below>
        end: <frame number in last file below
        files:
          - <list of files in order>
    target-touch:
      ros:
        start: 1608148955.2
        end: 1608149140.0
      gopro:
        start: <frame number in first file below>
        end: <frame number in last file below
        files:
          - <list of files in order>

  classical-telepresence:
    simon-says:
      ros:
        start:
        end:
      gopro:
        start: <frame number in first file below>
        end: <frame number in last file below
        files:
          - <list of files in order>
    target-touch:
      ros:
        start:
        end:
      gopro:
        start: <frame number in first file below>
        end: <frame number in last file below
        files:
          - <list of files in order>

  augmented-telepresence:
    simon-says:
      ros:
        start:
        end:
      gopro:
        start: <frame number in first file below>
        end: <frame number in last file below
        files:
          - <list of files in order>
    target-touch:
      ros:
        start:
        end:
      gopro:
        start: <frame number in first file below>
        end: <frame number in last file below
        files:
          - <list of files in order>

calibration:
ros:
  start:
  end:
gopro:
  start: <frame number in first file below>
  end: <frame number in last file below
  files:
    - <list of files in order>


bag-mapping:
  vid_color_data_upper: upper_realsense/color/image_raw_relay
  vid_color_info_upper: upper_realsense/color/camera_info
  vid_depth_data_upper: upper_realsense/depth/image_rect_raw_relay
  vid_depth_info_upper: upper_realsense/depth/camera_info
  vid_color_data_lower: lower_realsense/color/image_raw_relay
  vid_color_info_lower: lower_realsense/color/camera_info
  vid_depth_data_lower: lower_realsense/depth/image_rect_raw_relay
  vid_depth_info_lower: lower_realsense/depth/camera_info
'''
