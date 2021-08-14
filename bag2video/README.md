# Extract Video from RosBag

It is sometimes necessary to extract video from rosbags. This is designed to allow that.

## Features

- Multiple video source
- Single audio source
- Specified frame rate
- Runs on entire directory of bags
- Splits on no new images

## Arguments

bag2video -a [audio topic] -r [rows] -c [columns] -f [framerate (fps)] -s [split time (sec)] -d [target directory][video topics]

- audio topic: the topic with the audio to use
- rows (optional): the number of rows of video, defaults to the number of video topics
- columns (optional): the number of columns of video, defaults to 1
- framerate
- split time (optional): how long between receiving new images on any of the video topics to wait before splitting video. Defaults to infinity
- target directory: the directory with the bag files.
- video topics: which video topics to capture. Will be placed in order passed with row priority.
