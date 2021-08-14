from __future__ import division, print_function
import argparse
import math
import glob
import os
import logging
import cv2
# import time
import numpy as np
import rosbag
from cv_bridge import CvBridge
from tqdm import tqdm
from tqdm.contrib.logging import logging_redirect_tqdm
import moviepy.editor
import pdb

LOGGER = logging.getLogger(__name__)
VERBOSITY_OPTIONS = [logging.DEBUG, logging.INFO,
                     logging.WARNING, logging.ERROR, logging.CRITICAL]


def get_bag_filenames(target):
    """Find names of bag files

    Handles being given a directory or filename. In the case of the
    filename, as long as it is a valid filename, it is returned in
    a list as the single element. If given a directory, as long as
    it is a valid directory, then all bag files are found within
    that directory (non-recursively) and returned in a sorted list.

    Args:
        target: The target to look for (bag file path or path to a
                directory with bag files)

    returns: The list of bag files
    """
    bag_filenames = []
    target = os.path.expanduser(target).rstrip('/')+'/'
    LOGGER.info('Looking for bag file(s) at: %s', target)
    if os.path.isdir(target):
        LOGGER.debug("target is a directory")
        bag_filenames = glob.glob(target+"*.bag")
    elif os.path.isfile(target):
        LOGGER.debug("target is a file")
        bag_filenames = [target]
    else:
        raise FileNotFoundError("unable to find bag file")
    bag_filenames.sort()
    LOGGER.debug("found %i bag files", len(bag_filenames))
    return bag_filenames


def find_image_sizes(bag_filenames, video_topics):
    """Find the image sizes of a list of video topics in a list of bags

    Will for each video topic search through the bags in order for the
    first image and check its size. If no messages for the image topic
    are found, then that topic is ignored.

    Args:
        bag_filenames: A list of the filenames for the bags to search
        video_topics: The video topics to search for

    Returns: (list of video topics which were found,
              list of sizes (height, width, channels) for each of the found topics)
    """
    image_sizes = [[-1, -1]]*len(video_topics)
    bridge = CvBridge()
    for idx, topic in enumerate(video_topics):
        LOGGER.debug('looking for image size for: %s', topic)
        for bag_fn in bag_filenames:
            LOGGER.debug('looking in %s', bag_fn)
            bag = rosbag.Bag(bag_fn)
            try:
                _, msg, _ = next(bag.read_messages([topic]))
                height, width, channels = bridge.imgmsg_to_cv2(
                    msg, "bgr8").shape
                LOGGER.debug('found size (w x h): %i x %i', width, height)
                image_sizes[idx] = [height, width, channels]
                break
            except StopIteration:
                continue
    found_video_topics = []
    found_image_sizes = []
    for idx, _ in enumerate(image_sizes):
        if not image_sizes[idx] == -1:
            found_video_topics.append(video_topics[idx])
            found_image_sizes.append(image_sizes[idx])
    return (found_video_topics, found_image_sizes)


def get_tiled_image_size(image_sizes, cols):
    """Return the total size of a tiled image.

    Images will be tiled across rows (like text in a book)
    the final image width will be the maximum width of any individual
    row. Each row height will be the maximum height of any image in
    the row. The total image height will be the sum of row heights

    Args:
        image_sizes: a list of image sizes (width x height) ordered for
                     tiling
        cols: The number of columns in the final image

    Return: size (width x height) of the final image
    """
    row = 0
    col = 0
    width = 0
    height = 0
    row_width = 0
    row_height = 0
    for im_size in image_sizes:
        row_width += im_size[1]
        row_height = max(row_height, im_size[0])
        col += 1
        if col == cols:
            width = max(width, row_width)
            height += row_height
            row_width = 0
            row_height = 0
            col = 0
            row += 1

    return (width, height)


def bag2video(
        output,
        audio_topic,
        columns,
        framerate,
        split_time,
        target,
        audio_sample_rate,
        buffer_length,
        video_topics
):
    """Extract videos from bag files

    Can extract videos from bag files, including audio. Can extract
    multiple video topics and tile them. 

    Maintains a buffer for two reasons:
    1. some video topic messages may be delayed. 
    2. some timestamps might not have a frame from a topic. 

    The buffer will be 2*buffer_length + 1 the middle point is the 
    point to be saved. The zero index is the oldest currently saved
    frame and the len(buffer) index is the most recently read timestamp.

    Uses the timestamp on the image messages to get the right time. 
    For audio, there is no timestamp, so audio is taken in order
    and just concatenated.

    The audio start is either padded with silence or trimmed to match
    up with the video. 

    Whenever an image is being written, if there is nothing at the time
    index where the image should be pulled from, then the most recent 
    image captured is written. 

    Args:
        output:
        audio_topic:
        columns:
        framerate:
        split_time:
        target:
        audio_sample_rate:
        buffer_length:
        video_topics:
    """
    bag_filenames = get_bag_filenames(target)

    found_video_topics, found_image_sizes = find_image_sizes(
        bag_filenames, video_topics)

    topics = found_video_topics+[audio_topic]

    images = [[np.zeros((w, h, c), dtype='uint8')]*(buffer_length*2+1) for w,
              h, c in found_image_sizes]
    last_image = [np.zeros((w, h, c), dtype='uint8') for w,
                  h, c in found_image_sizes]
    audio = []
    audio_start_time = -1
    video_start_time = -1
    video_head = 0
    fps_increment = 1/framerate

    vid_frame_size = get_tiled_image_size(found_image_sizes, columns)
    LOGGER.info('Final image size will be (w x h): %i x %i',
                vid_frame_size[0], vid_frame_size[1])

    output = os.path.expanduser(output)

    output_extension = os.path.splitext(output)
    if output_extension[1] == '.mp4':
        fourcc = cv2.VideoWriter_fourcc(*'MP4V')
    elif output_extension[1] == '.avi':
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
    else:
        LOGGER.error(
            'invalid file extension on output: %s, extension: %s', output, output_extension)
        raise ValueError('invalid file extension on output arg')

    vid_writer = cv2.VideoWriter(
        output, fourcc, framerate, (vid_frame_size[0], vid_frame_size[1]))
    bridge = CvBridge()
    with logging_redirect_tqdm():
        for bag_fn in tqdm(bag_filenames):
            LOGGER.info('Reading %s', bag_fn)
            bag = rosbag.Bag(bag_fn)
            for topic, msg, msg_time, in bag.read_messages(topics=topics):
                if topic == audio_topic:
                    LOGGER.debug('Received new audio message')
                    if audio_start == -1:
                        audio_start = msg_time
                        LOGGER.info(
                            'Marking start of audio at %f', audio_start)
                    audio.append(msg.data)
                else:
                    LOGGER.debug('Received new image')
                    img_time = msg.header.stamp.to_sec()
                    if video_start_time == -1:
                        video_start_time = img_time
                        LOGGER.info('Marking start of video at %f',
                                    video_start_time)
                    img_idx = int(
                        round((img_time - video_start_time) * framerate))
                    LOGGER.debug('Image index: %i', img_idx)
                    if img_idx < 0:
                        continue
                    while img_idx > video_head+buffer_length:
                        head_idx = video_head % buffer_length
                        LOGGER.debug('writing out frame %i', head_idx)
                        img_to_write = []
                        for k_idx, img_list in enumerate(images):
                            LOGGER.debug('finding image for topic %i', k_idx)
                            for back in range(buffer_length):
                                LOGGER.debug('Looking back by %i', back)
                                back_idx = (head_idx-back) % buffer_length
                                if img_list[back_idx]:
                                    img_list[back_idx] = img_list[back_idx]
                                    break
                            if img_list[head_idx]
                            to_write = img_list[head_idx]
                            else:
                                to_write = last_image[k_idx]
                            img_to_write.append(to_write)
                            last_image[k_idx] = to_write
                            if img_list[head_idx] is None:
                                img_list[head_idx] = np.zeros(
                                    (
                                        found_image_sizes[k_idx][1],
                                        found_image_sizes[k_idx][0],
                                        3
                                    ),
                                    dtype='uint8')

                            img_list[head_idx] = None

                        video_head += 1

                        num_rows = int(math.ceil(len(img_to_write)/columns))
                        rows = [None]*num_rows
                        for r_idx in range(num_rows):
                            if columns > 1:
                                rows[r_idx] = cv2.hconcat(img_to_write[
                                    r_idx*columns:
                                    min(r_idx*columns+columns, len(img_to_write))
                                ])
                            else:
                                rows[r_idx] = img_to_write[r_idx]
                        out_img = cv2.vconcat(rows)

                        vid_writer.write(out_img)

                    head_idx = video_head % buffer_length
                    img_idx = ((img_idx-video_head) + head_idx) % buffer_length
                    images[topics.index(topic)][img_idx] = bridge.imgmsg_to_cv2(
                        msg, 'bgr8')

    vid_writer.release()
    LOGGER.info('finished writing video without audio to: %s', output)
    audio_video_offset = audio_start_time - video_start_time
    if audio_video_offset > 0:
        LOGGER.debug('padding start of audio with silence')
        audio = [0]*audio_video_offset*audio_sample_rate + audio
    elif audio_video_offset < 0:
        LOGGER.debug('trimming start of audio')
        audio = audio[int(audio_video_offset*audio_sample_rate):len(audio)]

    videoclip = moviepy.video.io.VideoFileClip.VideoFileClip(output)
    audioclip = moviepy.audio.AudioClip.AudioArrayClip(
        audio, fps=audio_sample_rate)
    video_with_audio = videoclip.set_audio(audioclip)
    video_with_audio.write_videofile(output)


if __name__ == '__main__':
    PARSER = argparse.ArgumentParser()

    PARSER.add_argument("-o", "--output", type=str,
                        help="where to save the video. Should be a filename ending in either " +
                        ".mp4 (uses mjpeg codec) or .avi (uses divx codec)")
    PARSER.add_argument("-a", "--audio_topic", type=str,
                        help="the audio topic to use in the video")
    PARSER.add_argument("-c", "--columns", type=int,
                        help="number of colums to use", default=1)
    PARSER.add_argument("-f", "--fps", type=int,
                        help="Number of frames per second to save", default=30)
    PARSER.add_argument("-s", "--split_time", type=float,
                        help="Number of seconds to wait before splitting video",
                        default=float('inf'))
    PARSER.add_argument("-t", "--target", type=str,
                        help="The target directory or file to convert. " +
                        "If given a directory, will convert all bag files sorted by filename")
    PARSER.add_argument("-z", "--audio_sample_rate", type=int,
                        default=16000, help="The sampling rate (hz) for the audio single " +
                        "(16,000 is the default for ros audio_common)")
    PARSER.add_argument("-b", "--buffer", type=int, default=10,
                        help="The internal image buffer to maintain to align images. A larger " +
                        "buffer can overcome cameras that are recording far from their capture " +
                        "time. Smaller values will use less memory")
    PARSER.add_argument('-v', "--verbosity", type=str,
                        choices=['debug', 'info',
                                 'warning', 'error', 'critical'],
                        default='warning', help='Verbosity level for logging')
    PARSER.add_argument("video_topics", type=str, nargs='+',
                        help="the video topics which you would like to include in the video")

    ARGS = PARSER.parse_args()

    V_IDX = ['debug', 'info', 'warning', 'error',
             'critical'].index(ARGS.verbosity)
    # LOGGER.setLevel(VERBOSITY_OPTIONS[V_IDX])
    # CH = logging.StreamHandler()
    # CH.setLevel(VERBOSITY_OPTIONS[V_IDX])
    # LOGGER.addHandler(CH)
    logging.basicConfig(level=VERBOSITY_OPTIONS[V_IDX])

    main(
        ARGS.output,
        ARGS.audio_topic,
        ARGS.columns,
        ARGS.fps,
        ARGS.split_time,
        ARGS.target,
        ARGS.audio_sample_rate,
        ARGS.buffer,
        ARGS.video_topics
    )
