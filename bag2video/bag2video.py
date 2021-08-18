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
import subprocess
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


def get_fourcc(fn):
    """Get fourcc code based on file extension

    .mp4 -> MP4V
    .av  -> XVID

    Args:
        fn: the file path/name which should have an extension
    """
    fn_extension = os.path.splitext(fn)
    if fn_extension[1] == '.mp4':
        fourcc = cv2.VideoWriter_fourcc(*'MP4V')
    elif fn_extension[1] == '.avi':
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
    else:
        LOGGER.error(
            'invalid file extension on: %s, extension: %s', fn, fn_extension)
        raise ValueError('invalid file extension on fn arg')
    return fourcc


def construct_vid_writer(output, framerate, size, num=None):
    """Constructs a video writer object

    Can add a number into the path to differentiate multiple files
    with the same name. If the num argument is none, does not add
    such a number

    Args:
        output: The path/name to write the video to
        framerate: The framerate for the video
        size: The frame size for the video
        num: The sub number for the video
    """
    output = os.path.expanduser(output)  # TODO implement num

    fourcc = get_fourcc(output)

    vid_writer = cv2.VideoWriter(output, fourcc, framerate, size)

    return vid_writer


def receive_audio_msg(audio, msg, msg_time):
    """Receive a new audio msg and store it

    Args:
        audio: The dictionary holding the audio data in field
               `data`, if need set start in `start` field. Will
               mutate the input dictionary.
        msg: The message to process
        msg_time: The time the message was received.
    """
    LOGGER.debug('Received new audio message')
    if audio['start'] == -1:
        audio['start'] = msg_time.to_sec()
        LOGGER.info(
            'Marking start of audio at %f', audio['start'])
    audio['data'] += msg.data


def set_video_start(video, img_time):
    """Set video start time if not already started.

    Will only make a change if the start time is not already
    set (as indicated by -1 being set)

    Args:
        video: The video dictionary which includes key `start`
        img_time: The time of the latest image.
    """
    if video['start'] == -1:
        video['start'] = img_time
        LOGGER.info('Marking start of video at %f',
                    video['start'])


def receive_video_msg(video, topic_idx, msg, bridge, vid_writer, columns):
    """Receive a new video msg and store it

    Args:
        video: The dictionary holding the video data in field
               `data`, if needed set the start in `start` field.
               Will mutate the input dictionary.
        topic_idx: The index of the topic we are receiving a message
                   from
        msg: The message
        bridge: Ros to CV2 bridge object
        vid_writer: Opencv bridge writer
        columns: The number of columns to plot out
    """
    LOGGER.debug('Received new image')
    img_time = msg.header.stamp.to_sec()
    set_video_start(video, img_time)
    img_idx = int(
        round((img_time - video['start']) * video['framerate']))
    LOGGER.debug('Image index: %i', img_idx)
    if img_idx < 0:
        return
    buffer_length = len(video['images'][0])
    while img_idx > video['head']+buffer_length:
        head_idx = video['head'] % buffer_length
        LOGGER.debug(
            'writing out frame %i at buffer idx %i', video['head'], head_idx)
        img_to_write = retrieve_img_to_write(video)
        video['head'] += 1

        out_img = stitch_image(img_to_write, columns)
        vid_writer.write(out_img)

    insert_image(video, msg, topic_idx, img_idx, bridge)


def retrieve_img_to_write(video):
    """Retrieve the next image list to write from the video dictionary.

    For each topic, looks for an image at the current buffer head to
    write out. If none is found, then use the last stored image

    Args:
        video: The video dictionary with fields `images` storing the
               images `head` pointing to the head of the buffer and
               `last_image` pointing to the last image dropped off
               of the buffer
    """
    head_idx = video['head'] % len(video['images'][0])
    img_to_write = [None]*len(video['images'])
    for k_idx, img_list in enumerate(video['images']):
        LOGGER.debug(
            'writing out image for topic %i', k_idx)
        if img_list[head_idx] is not None:
            LOGGER.debug('found image in buffer')
            img_to_write[k_idx] = img_list[head_idx]
            video["last_image"][k_idx] = img_list[head_idx]
        else:
            LOGGER.debug('using last image dropped from buffer')
            img_to_write[k_idx] = video['last_image'][k_idx]
        img_list[head_idx] = None
    return img_to_write


# TODO: implement support for variable shaped images
def stitch_image(img_to_write, columns):
    """Stitch list of images into a cohesive image

    Stitchs across rows first.

    Args:
        img_to_write: The list of images to write
        columns: The number of columns
    """
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
    return out_img


def add_audio(fn, video, audio):
    """Add audio track to video

    Will take the already written out video file, load it in,
    pad or trim the start of the audio to match the video
    start, add the audio, and write back out the video.

    Args:
        fn: The filename of the video to work with
        video: Video dictionary with field `start`
        audio: Audio dictionary with fields `start`, `data`
               and `sample_rate`
    """
    mp3_fn = '{}-tmp.mp3'.format(os.path.splitext(fn)[0])
    with open(mp3_fn, 'w+') as mp3_file:
        mp3_file.write(''.join(audio['data']))

    sp_fn = os.path.splitext(fn)
    tmp_vid_fn = '{}-tmp{}'.format(sp_fn[0], sp_fn[1])

    audio_video_offset = audio['start'] - video['start']
    if audio_video_offset >= 0:
        LOGGER.info('starting video %f seconds before audio',
                    audio_video_offset)
        command = 'ffmpeg -i {} -itsoffset {} -i {} -map 0:v -map 1:a -c:v copy {}'.format(
            tmp_vid_fn, audio_video_offset, mp3_fn, fn)
    else:
        LOGGER.info('starting audio %f seconds before video',
                    abs(audio_video_offset))
        command = 'ffmpeg -i {} -itsoffset {} -i {} -map 0:a -map 1:v -c:v copy {}'.format(
            mp3_fn, abs(audio_video_offset), tmp_vid_fn, fn)
    LOGGER.debug('Combining video and audio with: %s', command)
    return_code = subprocess.call(command, shell=True)
    if return_code != 0:
        raise RuntimeError(
            'failed at running ffmpeg to combine audio and video')


def insert_image(video, msg, topic_idx, img_idx, bridge):
    """Insert image from message into buffer

    Args:
        video: The video dictionary with fields `images` and
               `head`. This will be mutated
        msg: The message with the image to insert
        topic_idx: The topic index for the message
        img_idx: The index of this image (based on its time)
        bridge: Bridge to convert from ros msg to opencv img
    """
    buffer_length = len(video['images'][0])
    head_idx = video['head'] % buffer_length
    img_buffer_idx = (
        (img_idx-video['head']) + head_idx) % buffer_length
    LOGGER.debug('received image from topic %i', topic_idx)
    video['images'][topic_idx][img_buffer_idx] = bridge.imgmsg_to_cv2(
        msg, 'bgr8')


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

    Maintains 2 buffers:
    1. A buffer which is <arg: buffer_length> which is meant to handle
       the potential offset of capture times for different topics.
    2. A buffer which stores the last image that was removed from the
       previous buffer.

    Uses the timestamp on the image messages to get the right time.
    For audio, there is no timestamp, so audio is taken in order
    and just concatenated.

    The audio start is either padded with silence or trimmed to match
    up with the video.

    Whenever an image is being written, if there is nothing at the time
    index where the image should be pulled from, then the most recent
    image captured is written.

    The video can be optionally split anytime no frames have been read
    for a specified amount of time.

    Args:
        output: The filepath to save the complete videos to. Should have
                the full path, filename, and extension.
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

    audio = {'start': -1, 'data': [], 'sample_rate': audio_sample_rate}
    video = {'start': -1, 'head': 0, 'framerate': framerate,
             'images': [[None]*buffer_length for _ in found_image_sizes],
             'last_image': [np.zeros((w, h, c), dtype='uint8') for w, h, c in found_image_sizes]}

    vid_frame_size = get_tiled_image_size(found_image_sizes, columns)
    LOGGER.info('Final image size will be (w x h): %i x %i',
                vid_frame_size[0], vid_frame_size[1])

    sp_fn = os.path.splitext(output)
    tmp_vid_fn = '{}-tmp{}'.format(sp_fn[0], sp_fn[1])
    vid_writer = construct_vid_writer(
        tmp_vid_fn, framerate, vid_frame_size[0:2])
    bridge = CvBridge()

    with logging_redirect_tqdm():
        for bag_fn in tqdm(bag_filenames):
            LOGGER.info('Reading %s', bag_fn)
            bag = rosbag.Bag(bag_fn)
            for topic, msg, msg_time, in bag.read_messages(topics=topics):
                if topic == audio_topic:
                    receive_audio_msg(audio, msg, msg_time)
                else:
                    topic_idx = topics.index(topic)
                    receive_video_msg(video, topic_idx, msg,
                                      bridge, vid_writer, columns)

    vid_writer.release()
    LOGGER.info('finished writing video without audio to: %s', tmp_vid_fn)

    add_audio(output, video, audio)


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
    PARSER.add_argument("-b", "--buffer", type=int, default=100,
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

    bag2video(
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
