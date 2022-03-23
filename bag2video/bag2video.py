#!/usr/bin/env python
"""A module for exporting video with audio from ROS bag files

Primarily designed to be used as a command line tool. It should
come with a bash script and dockerfile to make that easy."""

from __future__ import division, print_function
import argparse
import math
import glob
import os
import logging
import subprocess
import sys
import cv2
import numpy as np
import rosbag
from cv_bridge import CvBridge
from tqdm import tqdm
from tqdm.contrib.logging import logging_redirect_tqdm
# from pydub import AudioSegment
# import io
from streamp3 import MP3Decoder
import scipy.io.wavfile
from common import img_overlays

LOGGER = logging.getLogger(__name__)
VERBOSITY_OPTIONS = [logging.DEBUG, logging.INFO,
                     logging.WARNING, logging.ERROR, logging.CRITICAL]

# kilobits/sec for MPEG 2 Layer III
BITRATES = np.array([0, 8, 16, 24, 32, 40, 48, 56,
                     64, 80, 96, 112, 128, 144, 160])


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
    target = os.path.expanduser(target).rstrip('/')
    LOGGER.info('Looking for bag file(s) at: %s', target)
    if os.path.isdir(target+'/'):
        LOGGER.debug("target is a directory")
        bag_filenames = glob.glob(target+'/'+"*.bag")
    elif os.path.isfile(target):
        LOGGER.debug("target is a file")
        bag_filenames = [target]
    else:
        raise IOError("unable to find bag file")
    bag_filenames.sort()
    LOGGER.info("found %i bag files:", len(bag_filenames))
    for filename in bag_filenames:
        LOGGER.info("\t%s", filename)
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
    image_sizes = [None]*len(video_topics)
    bridge = CvBridge()
    for idx, topic in enumerate(video_topics):
        LOGGER.debug('looking for image size for: %s', topic)
        for bag_filename in bag_filenames:
            LOGGER.debug('looking in %s', bag_filename)
            bag = rosbag.Bag(bag_filename)
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
        if not image_sizes[idx] is None:
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


def get_fourcc(filename):
    """Get fourcc code based on file extension

    .mp4 -> MP4V
    .av  -> XVID

    Args:
        filename: the file path/name which should have an extension
    """
    filename_extension = os.path.splitext(filename)
    if filename_extension[1] == '.mp4':
        fourcc = cv2.VideoWriter_fourcc(*'MP4V')
    elif filename_extension[1] == '.avi':
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
    else:
        LOGGER.error(
            'invalid file extension on: %s, extension: %s', filename, filename_extension)
        raise ValueError('invalid file extension on filename arg')
    return fourcc


def construct_vid_writer(output, framerate, size):
    """Constructs a video writer object

    Can add a number into the path to differentiate multiple files
    with the same name. If the num argument is none, does not add
    such a number

    Args:
        output: The path/name to write the video to
        framerate: The framerate for the video
        size: The frame size for the video
    """
    output = os.path.expanduser(output)

    fourcc = get_fourcc(output)

    vid_writer = cv2.VideoWriter(output, fourcc, framerate, size)

    return vid_writer


def receive_audio_msg(audio, msg, msg_time):
    """Receive a new audio msg and store it

    TODO: Handle messages that are dropped by placing audio into an array
          based on the time of receipt

    Args:
        audio: The dictionary holding the audio data in field
               `data`, if need set start in `start` field. Will
               mutate the input dictionary.
        msg: The message to process
        msg_time: The time the message was received.
    """
    # ipdb.set_trace()
    # crazy mp3 decoders need some nonesense
    # out2=AudioSegment.from_file(io.BytesIO(msg.data+msg.data))
    # len(out2.raw_data)=2304
    # mp3_decoder = MP3Decoder(msg.data+msg.data+msg.data)
    # out=[chunk for chunk in mp3_decoder]
    # len(out[0])=1152
    # out2 and out 1 are almost (but not quite) identical

    LOGGER.debug('Received new audio message')
    if audio['start'] == -1:
        audio['start'] = msg_time.to_sec()
        LOGGER.info(
            'Marking start of audio at %f', audio['start'])
    audio['data2time'].append([len(audio['data']), msg_time.to_sec()])
    audio['data'] += msg.data


def set_video_start(video, img_time, msg_time):
    """Set video start time if not already started.

    Will only make a change if the start time is not already
    set (as indicated by -1 being set)

    Args:
        video: The video dictionary which includes key `start`
        img_time: The time of the latest image.
    """
    if video['start'] == -1:
        video['start'] = img_time
        video['first_msg_time'] = msg_time
        LOGGER.info('Marking start of video at %f',
                    video['start'])


def receive_video_msg(video, topic_idx, msg, msg_time, bridge, vid_writer, columns):
    # pylint: disable=too-many-arguments
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
    LOGGER.debug('Received new image with message time: %f', msg_time.to_sec())
    img_time = msg.header.stamp.to_sec()
    set_video_start(video, img_time, msg_time.to_sec())
    img_idx = int(
        round((img_time - video['start']) * video['framerate']))
    LOGGER.debug('Image index: %i, header time: %f', img_idx, img_time)
    if img_idx < 0:
        LOGGER.warning('image found before start, discarding')
        return
    buffer_length = len(video['images'][0])
    while img_idx >= video['head']+buffer_length:
        write_image(video, vid_writer, columns)

    insert_image(video, msg, topic_idx, img_idx, bridge)


def write_image(video, vid_writer, columns):
    """Write the head image out.

    Args:
        video: The video dictionary with keys `head` and `images`
        vid_writer: The cv2 video writer object
        columns: The number of columns the final image should have
    """
    buffer_length = len(video['images'][0])
    head_idx = video['head'] % buffer_length
    LOGGER.debug(
        'writing out frame %i at buffer idx %i', video['head'], head_idx)
    img_to_write = retrieve_img_to_write(video)

    out_img = stitch_image(img_to_write, columns)
    add_timestamp(out_img, video)
    vid_writer.write(out_img)


def add_timestamp(img, video):
    """Add a timestamp to the output video

    Args:
        img: the image to draw over
        video: the video data structure used to figure out the time
    """
    time_str = str(video['first_msg_time'] +
                   (video['head']*(1/video['framerate'])))
    LOGGER.debug('writing time on frame: %s', time_str)
    img_overlays.draw_text(img, time_str)


def retrieve_img_to_write(video):
    """Retrieve the next image list to write from the video dictionary and
    drop it out of the buffer.

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
    video['head'] += 1
    return img_to_write


def stitch_image(img_to_write, columns):
    """Stitch list of images into a cohesive image

    Stitchs across rows first.

    Args:
        img_to_write: The list of images to write
        columns: The number of columns
    """
    LOGGER.debug('Stitching images with sizes: %s', str(
        [np.shape(img) for img in img_to_write]))
    num_rows = int(math.ceil(len(img_to_write)/columns))
    rows = [None]*num_rows
    for r_idx in range(num_rows):
        if columns > 1:
            # TODO: make sure that images that are being horizontally
            #       concatenated have the same height
            rows[r_idx] = cv2.hconcat(img_to_write[
                r_idx*columns:
                min(r_idx*columns+columns, len(img_to_write))
            ])
        else:
            rows[r_idx] = img_to_write[r_idx]
    # Need to make sure that every row has the same width.
    # find max width and
    max_w = np.max([row.shape[1] for row in rows])
    for row_idx, _ in enumerate(rows):
        left_padding = int(np.floor((max_w - np.shape(rows[row_idx])[1])/2))
        right_padding = int(np.ceil((max_w - np.shape(rows[row_idx])[1])/2))
        if left_padding > 0 or right_padding > 0:
            rows[row_idx] = cv2.copyMakeBorder(
                rows[row_idx],
                0, 0,
                left_padding, right_padding,
                cv2.BORDER_CONSTANT
            )
    out_img = cv2.vconcat(rows)
    return out_img


def construct_pcm(audio):
    """Construct PCM data, appropriately spaced in time, given the provided
    audio object.

    Args:
        audio: A dictionary with fields 'data' (mp3 data received) and
               'data2time' (which maps the time of message receipt and
               data messages to each other)

    """
    # find headers to be sure that a header is valid, one must check
    # the rest of the frame. OMG, who designed MP3? why is there no
    # reserved word? Like limit the data to only allow 0x1*bit_depth at
    # the start of the header. Dumb
    data_arr = np.array(audio['data'])
    headers = np.where((data_arr[0:-3] == 255) *
                       (data_arr[1:-2] == 243) *
                       (np.right_shift(data_arr[2:-1], 4) != 0x0F) *
                       (np.bitwise_and(data_arr[2:-1], 0x0C) == 0x08) *
                       (data_arr[3:] == 196))[0]
    padding = (data_arr[headers+2] & 0b10)
    data2time = np.array(audio['data2time'])
    if not np.isin(data2time[:, 0], headers).all():
        raise RuntimeError(
            'Some messages did not start with well formed headers')
        # TODO: handle messed up messages gracefully
        # TODO: check frame lengths
    frame_size = ((576 / 8 * (BITRATES[data_arr[headers+2] >> 4]))/16)+padding
    # always 576 samples/frame for V2 Layer III stereo and 1152 for mono?

    # PyDub
    # For whatever reason FFMPEG expects frames of framesize + header size.
    # I think it should just be frame size, I could pad it or something, but
    # that would change the underlying compression output:
    # ipdb> AudioSegment.from_file(io.BytesIO(bytes(audio['data'][0:432+4])))

    # streamp3
    # creates chunks of 16 bit PCM, but ends up missing two chunks?
    # trying to read with only first frame or only last frame just gives nothing back.
    padding = audio['data'][headers[0]:headers[0]+4]+[0]*int(frame_size[0]-4)
    # padding is needed to make the underlying stuff works. Here is an idea
    # as to why that might be: https://thebreakfastpost.com/2016/11/26/
    #                          mp3-decoding-with-the-mad-library-weve-all-been-doing-it-wrong/
    mp3_decoder = MP3Decoder(bytes(padding+audio['data']+padding))
    all_chunks = list(mp3_decoder)
    # the first frame will just be the padding coming back (doesn't make sense to me either..)
    all_chunks = all_chunks[1:]
    #

    # int_chunks = [[(chunks_arr_elem[idx*2] << 8) | chunks_arr_elem[idx*2+1]
    #                for idx in range(int(len(chunks_arr_elem)/2))]
    #               for chunks_arr_elem in all_chunks]
    assert mp3_decoder.sample_rate == 16000
    assert mp3_decoder.num_channels == 1
    # assert (~(np.array([len(chunk) for chunk in int_chunks]) != 576)).all()
    LOGGER.info(
        'bit rate: %i, sample rate: %i, num channels: %i',
        mp3_decoder.bit_rate, mp3_decoder.sample_rate, mp3_decoder.num_channels)
    LOGGER.info(
        '%i valid headers found, but LAME only reads %i chunks', len(headers), len(all_chunks))
    filled_data = []
    end_time = audio['start']
    for frame_idx, this_chunk in enumerate(all_chunks):
        data2time = audio['data2time'][frame_idx]
        diff = data2time[1] - end_time
        if diff > 0:
            add_samples = int(diff*16000)
            end_time += add_samples/16000
            filled_data += (bytes([0, 0]*add_samples))

        filled_data += (this_chunk)
        end_time += 576/16000

    # length = len(filled_data)/mp3_decoder.sample_rate

    return [int.from_bytes(bytes(filled_data[idx*2:idx*2+2]),
                           byteorder='little', signed=True)
            for idx in range(int(len(filled_data)/2))]


def add_audio(filename, video, audio, idx):
    """Add audio track to video

    Will take the already written out video file, load it in,
    pad or trim the start of the audio to match the video
    start, add the audio, and write back out the video.

    Good sources on MP3 file format:
    - https://www.codeproject.com/Articles/8295/MPEG-Audio-Frame-Header#ModeExt
    - http://www.mp3-tech.org/programmer/frame_header.html

    Args:
        filename: The filename of the video to work with
        video: Video dictionary with field `start`
        audio: Audio dictionary with fields `start`, `data`
               and `sample_rate`
        idx: which number video is being written
    """
    pcm_data = construct_pcm(audio)

    tmp_vid_filename = get_tmp_vid_filename(filename)
    if audio['start'] == -1:
        LOGGER.warning('no audio found')
        os.rename(tmp_vid_filename, get_idx_vid_filename(filename, idx))
        return

    audio_filename = f'{os.path.splitext(filename)[0]}-tmp.mp3'
    # if sys.version_info[0] == 3:
    #     with open(audio_filename, 'w+b') as mp3_file:
    #         mp3_file.write(bytes(audio['data']))
    # else:
    #     with open(audio_filename, 'w+') as mp3_file:
    #         mp3_file.write(''.join(audio['data']))
    scipy.io.wavfile.write(audio_filename, 16000,
                           np.asarray(pcm_data, dtype='int16'))

    audio_video_offset = audio['start'] - video['first_msg_time']
    if audio_video_offset >= 0:
        LOGGER.info('starting video %f seconds before audio',
                    audio_video_offset)
        command = (f'ffmpeg -y -i {tmp_vid_filename} -itsoffset {audio_video_offset} ' +
                   f'-i {audio_filename} -map 0:v -map 1:a -c:v copy ' +
                   f'{get_idx_vid_filename(filename, idx)}')
    else:
        LOGGER.info('starting audio %f seconds before video',
                    abs(audio_video_offset))
        command = (f'ffmpeg -y -i {audio_filename} -itsoffset {abs(audio_video_offset)} ' +
                   f'-i {tmp_vid_filename} -map 0:a -map 1:v -c:v copy ' +
                   f'{get_idx_vid_filename(filename, idx)}')
    LOGGER.debug('Combining video and audio with: %s', command)
    return_code = subprocess.call(command, shell=True)
    if return_code != 0:
        raise RuntimeError(
            'failed at running ffmpeg to combine audio and video')
    os.remove(audio_filename)
    os.remove(tmp_vid_filename)


def get_tmp_vid_filename(filename):
    """Generate a temporary video filename based on the passed in filename

    Args:
        filename: The root filename to add temp into
    """
    sp_filename = os.path.splitext(filename)
    tmp_vid_filename = f'{sp_filename[0]}-tmp{sp_filename[1]}'
    return tmp_vid_filename


def get_idx_vid_filename(filename, idx):
    """Generate a numbered video filename based on the passed in filename

    Args:
        filename: The root filename to add temp into
        idx: the index to add into the filename
    """
    sp_filename = os.path.splitext(filename)
    idx_filename = f'{sp_filename[0]}-{idx}{sp_filename[1]}'
    return idx_filename


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
    if video['images'][topic_idx][img_buffer_idx] is not None:
        LOGGER.warning('overwriting an existing image')
    video['images'][topic_idx][img_buffer_idx] = cv2.resize(bridge.imgmsg_to_cv2(
        msg, 'bgr8'), tuple(video["sizes"][topic_idx][1:None:-1]))


# pylint: disable=too-many-locals
# pylint: disable=too-many-arguments
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

    video_sizes = [vt.split(':')[1] if len(vt.split(':'))
                   > 1 else None for vt in video_topics]
    video_topics = [vt.split(':')[0] for vt in video_topics]

    found_video_topics, found_image_sizes = find_image_sizes(
        bag_filenames, video_topics)

    for idx, video_size in enumerate(video_sizes):
        if video_size is not None:
            # (height, width, channels)
            height, width = video_size.split('x')
            found_image_sizes[idx][0] = int(height)
            found_image_sizes[idx][1] = int(width)

    topics = found_video_topics+[audio_topic]

    LOGGER.info('Will be processing with topics:')
    for topic in topics:
        LOGGER.info('\t%s', topic)

    audio = construct_audio(audio_sample_rate)
    video = construct_video(framerate, buffer_length, found_image_sizes)
    last_vid_time = -1

    vid_frame_size = get_tiled_image_size(found_image_sizes, columns)
    LOGGER.info('Final image size will be (w x h): %i x %i',
                vid_frame_size[0], vid_frame_size[1])

    tmp_vid_filename = get_tmp_vid_filename(output)
    vid_writer = construct_vid_writer(
        tmp_vid_filename, framerate, vid_frame_size[0:2])
    bridge = CvBridge()

    idx = 0

    with logging_redirect_tqdm():
        for bag_filename in tqdm(bag_filenames):
            LOGGER.info('Reading %s', bag_filename)
            bag = rosbag.Bag(bag_filename)
            for topic, msg, msg_time, in bag.read_messages(topics=topics):
                if last_vid_time != -1 and (msg_time.to_sec() - last_vid_time) > split_time:
                    LOGGER.info('splitting video')
                    write_out(video, audio, output, vid_writer, columns, idx)
                    idx += 1
                    last_vid_time = -1
                    audio = construct_audio(audio_sample_rate)
                    video = construct_video(
                        framerate, buffer_length, found_image_sizes)
                    vid_writer = construct_vid_writer(
                        tmp_vid_filename, framerate, vid_frame_size[0:2])
                if topic == audio_topic:
                    receive_audio_msg(audio, msg, msg_time)
                else:
                    topic_idx = topics.index(topic)
                    last_vid_time = msg_time.to_sec()
                    receive_video_msg(video, topic_idx, msg, msg_time,
                                      bridge, vid_writer, columns)

        write_out(video, audio, output, vid_writer, columns, idx)


def construct_audio(audio_sample_rate):
    """Construct the audio dictionary, empty

    Args:
        audio_sample_rate: The sampling rate for the audio
    """
    audio = {'start': -1,
             'data': [],
             'data2time': [],
             'sample_rate': audio_sample_rate}
    return audio


def construct_video(framerate, buffer_length, image_sizes):
    """Construct the video dictionary, empty

    Args:
        framerate: The video framerate
        buffer_length: The length of the buffer for video to maintain
        image_sizes: The sizes of the images that will be inserted
    """
    video = {'start': -1, 'head': 0, 'framerate': framerate,
             'images': [[None]*buffer_length for _ in image_sizes],
             'last_image': [np.zeros((w, h, c), dtype='uint8')
                            for w, h, c in image_sizes],
             'sizes': image_sizes}
    return video


# pylint: disable=too-many-arguments
def write_out(video, audio, filename, vid_writer, columns, idx):
    """Write out the remaining data, including any un added images
    and the audio data.

    Will create a intermediate temporary file in the progress.

    Args:
        video: The video data dictionary
        audio: The audio data dictionary
        filename: The target output filename
        vid_writer: The OpenCV video writer object
        columns: The number of columns to use when writing out the video
        idx: the index to add into the filename
    """
    while not all(
            all(img is None for img in video['images'][cam])
            for cam in range(len(video['images']))
    ):
        write_image(video, vid_writer, columns)

    vid_writer.release()

    if not video['start'] == -1:
        tmp_vid_filename = get_tmp_vid_filename(filename)
        LOGGER.info('finished writing video without audio to: %s',
                    tmp_vid_filename)

        try:
            add_audio(filename, video, audio, idx)
        except Exception as exc:  # pylint: disable=broad-except
            os.rename(tmp_vid_filename, get_idx_vid_filename(filename, idx))
            LOGGER.error('There was an exception while adding audio: %s', exc)


if __name__ == '__main__':
    print(f'got args: {sys.argv}')
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
                        help="the video topics which you would like to include in the video, " +
                        "this can come in as just the name of the topic in which case the video " +
                        "size will be checked (first image size in topic used) or can come in as " +
                        "`<video topic>:<widhth>x<height>` " +
                        "for example: `/lower_realsense/color/image_raw_relay:1920x1080` " +
                        "The benefit of passing the size in manually is that any necessary " +
                        "scaling will be handled and if no images are found, then a black " +
                        "space [not implemented] will be left for the missing image, and " +
                        "if topics change size, the size you want will be used")
    # TODO: implement direct passing of width and height of video

    ARGS = PARSER.parse_args()

    print(ARGS)

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
