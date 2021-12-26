#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# pylint: disable=too-many-lines
"""
Created on Thu May 28 23:21:43 2020

@author: gsuveer, mjsobrep

Module to take a rosbag and output a hdf5 database or append to existing hdf5
database.

Attributes:
    pre_processing_bag_file  : Name of the bag file to convert to HDF5 Database
    hdf5_filename : Name of output file
    Mode: 'a' for append, 'w' for write
    experiment_id: Initial namespace for HDF5

TODO
    Figure out if chunk size improves speed. Currently letting API call infer.
    Add additional topics apart from image topics
"""

import pathlib
import copy
import os
import json
import numpy as np
import h5py
import rospy
import rosbag
from cv_bridge import CvBridge
from tqdm import tqdm
from rospy_message_converter import message_converter
from rospy_message_converter import json_message_converter

INITIAL_SIZE = 0
CHUNK_SIZE = 100

DIAGNOSTIC_DTYPE = np.dtype([
    ('level', np.ubyte),
    ('name', h5py.string_dtype()),
    ('message', h5py.string_dtype())
])
FACE_DTYPE = np.dtype([
    ('eye_name', h5py.string_dtype()),
    ('mouth_name', h5py.string_dtype()),
    ('description', h5py.string_dtype())
])
GAME_ACTION_DTYPE = np.dtype([
    ('speech', h5py.string_dtype()),
    ('step_id', int),
    ('targets', h5py.string_dtype())
])
NET_DTYPE = np.dtype([
    ('link_quality', float),
    ('signal_strength', float),
    ('network_ssid', h5py.string_dtype()),
    ('ip_addr', h5py.string_dtype())
])
MOVE_RESULT_DTYPE = np.dtype([
    ('completed', bool),
    ('positional_error', float),
    ('status', np.uint8),
    ('text', h5py.string_dtype())
])
MOVE_FEEDBACK_DTYPE = np.dtype([
    ('time_elapsed', float),
    ('time_remaining', float),
    ('move_number', np.uint16)
])
ROSOUT_DTYPE = np.dtype([
    ('level', np.byte),
    ('name', h5py.string_dtype()),
    ('msg', h5py.string_dtype()),
    ('file', h5py.string_dtype()),
    ('function', h5py.string_dtype()),
    ('line', np.uint32)
])
SOUND_FEEDBACK_DTYPE = np.dtype([
    ('playing', bool),
    ('stamp', np.double)
])
SOUND_GOAL_DTYPE = np.dtype([
    ('command', np.int8),
    ('volume', float),
    ('arg', h5py.string_dtype()),
    ('arg2', h5py.string_dtype())
])
QUATERNION_DTYPE = np.dtype([
    ('x', np.double),
    ('y', np.double),
    ('z', np.double),
    ('w', np.double)
])
VECTOR3_DTYPE = np.dtype([
    ('x', np.double),
    ('y', np.double),
    ('z', np.double)
])
TRANSFORM_DTYPE = np.dtype([
    ('translation', VECTOR3_DTYPE),
    ('rotation', QUATERNION_DTYPE)
])
TRANSFORM_STAMPED_DTYPE = np.dtype([
    ('child_frame_id', h5py.string_dtype()),
    ('transform', TRANSFORM_DTYPE),
    ('parent_frame_id', h5py.string_dtype())
])
REALSENSE_EXTRINSICS_DTYPE = np.dtype([
    ('rotation', np.double, 9),
    ('translation', np.double, 3)
])
TTS_STATE_DTYPE = np.dtype([
    ('state', np.int8),
    ('text', h5py.string_dtype())
])
SPEECH_GOAL_DTYPE = np.dtype([
    ('text', h5py.string_dtype()),
    ('metadata', h5py.string_dtype())
])
TWIST_DTYPE = np.dtype([
    ('linear', VECTOR3_DTYPE),
    ('angular', VECTOR3_DTYPE)
])
IMU_DTYPE = np.dtype([
    ('orientation', QUATERNION_DTYPE),
    ('orientation_covariance', np.double, 9),
    ('angular_velocity', VECTOR3_DTYPE),
    ('angular_velocity_covariance', np.double, 9),
    ('linear_acceleration', VECTOR3_DTYPE),
    ('linear_acceleration_covariance', np.double, 9)
])
POSE_COVARIANCE_DTYPE = np.dtype([
    ('pose', TRANSFORM_DTYPE),
    ('covariance', np.double, 36)
])
TWIST_COVARIANCE_DTYPE = np.dtype([
    ('twist', TWIST_DTYPE),
    ('covariance', np.double, 36)
])
ODOMETRY_DTYPE = np.dtype([
    ('child_frame_id', h5py.string_dtype()),
    ('pose', POSE_COVARIANCE_DTYPE),
    ('twist', TWIST_COVARIANCE_DTYPE),
    ('parent_frame_id', h5py.string_dtype())
])


def match_video_2_data(meta_data):
    """Create a dictionary mapping from video topic names to their corresponding info topic names

    Args:
        meta_data: the metadata dictionary

    Returns: A dictionary with keys video data topic names and values info topic names
    """
    info_topics = filter(lambda x: 'vid' and 'info' in x,
                         meta_data['bag-mapping'].keys())
    data_topics = filter(lambda x: 'vid' and 'data' in x,
                         meta_data['bag-mapping'].keys())
    data_info_mapping = {}
    for topic in data_topics:
        topic_root = topic.replace('/data', '')
        target_name = topic_root+'/info'
        if target_name in info_topics:
            data_info_mapping[topic_root] = target_name
        else:
            rospy.logerr('video topic without matching info topic: %s', topic)
    return data_info_mapping


def load_bag_file(bag_filename):
    """Load a bag file from disk

    Args:
        bag_filename: the filename of the bag file

    Returns: The bag file
    """
    if not bag_filename == '':
        try:
            bag_file = rosbag.Bag(bag_filename)
        except:  # pylint: disable=bare-except
            rospy.loginfo('BAG FILE COULD NOT BE READ')
            raise
    else:
        rospy.logerr('No Argument provided for Bag Filename')
        return None

    topics = bag_file.get_type_and_topic_info()[1].keys()
    rospy.logdebug('Successfully read bag file with topics:')
    for topic in topics:
        rospy.logdebug('\t\t%s', topic)
    return bag_file


def get_topic_info(data_info_mapping, bag_file, meta_data):
    """Get the first info message for each video file

    Args:
        data_info_mapping: a dictionary mapping from data topic names to info topic names
        bag_file: The bag file to extract from
        meta_data: The meta data as a dictionary

    Returns: A dictionary with the information in the first message for each video,
             keyed on the video topic name
    """
    topic_info = {}
    for vid_topic in data_info_mapping:
        true_topic = meta_data['bag-mapping'][data_info_mapping[vid_topic]]
        rospy.loginfo(
            'Extracting meta info from standard video topic: %s', vid_topic)
        rospy.loginfo('\t\tmapped to real info topic: %s', true_topic)
        try:
            _, msg, _ = next(bag_file.read_messages([true_topic]))
        except StopIteration:
            topic_info[vid_topic] = None
            continue
        topic_info[vid_topic] = msg
    return topic_info


def get_realsense_extrinsics(bag_file):
    """Extracts extrinsics for all realsense cameras

    Args:
        bag_file: the opened bag file

    Returns: A dictionary with keys that are the names of the camera
             ex: 'upper_realsense' and values that are a message of type
             realsense2_camera/Extrinsics. These messages have three fields
             `header`, `rotation`, `translation`. The rotation is 9 element
             list, the translation is a 3 element list.
    """
    topics = bag_file.get_type_and_topic_info()[1].keys()
    realsense_topics = filter(lambda val: 'realsense' in val, topics)
    extrinsics_topics = filter(
        lambda val: 'extrinsics' and 'depth_to_color' in val, realsense_topics)

    extrinsics = {}
    for topic in extrinsics_topics:
        rospy.loginfo('extracting extrinsics from: %s', topic)
        try:
            _, msg, _ = next(bag_file.read_messages([topic]))
        except StopIteration:
            continue
        if 'lower' in topic:
            cam = 'lower'
        elif 'upper' in topic:
            cam = 'upper'
        extrinsics[cam] = {'rotation': msg.rotation,
                           'translation': msg.translation}
    return extrinsics


def load_hdf_file(out_dir):
    """Load HDF file, creating it if needed

    Args:
        out_dir: The directory to save to

    Returns: The HDF5 File data object
    """
    hdf5_fn_full = pathlib.Path(out_dir, 'full_data-vid').with_suffix('.hdf5')
    pth = pathlib.Path(hdf5_fn_full)
    pth.parent.mkdir(parents=True, exist_ok=True)
    try:
        hdf5_database = h5py.File(hdf5_fn_full, 'a')
    except:  # pylint: disable=bare-except
        rospy.logerr(
            'HDF5 Database COULD NOT BE READ/CREATED: %s', hdf5_fn_full)
        raise
    return hdf5_database


def add_video_topics(hdf5_database,  data_info_mapping, meta_data, topic_info, extrinsics):
    """Create necessary databases in the hdf5 file for video
    and fill in atrributes using the topic info where relevant

    Args:
        hdf5_database: The hdf5_database to add to
        data_info_mapping: The mapping between video data topics and info data topics
        meta_data: The meta data as a dictionary
        topic_info: The data from the info topics in a dictionary keyed on the video topic name
    """
    for root_topic in data_info_mapping:
        vid_topic = root_topic+'/data'
        rospy.loginfo(
            '\tParsing for standard topic : %s' % (vid_topic))
        raw_topic = meta_data['bag-mapping'][vid_topic]
        rospy.loginfo(
            '\t\traw topic : %s' % (raw_topic))
        topic_meta_info = topic_info[root_topic]
        if topic_meta_info:
            vid_topic_time = root_topic + '/time'
            if 'depth' in vid_topic and vid_topic not in hdf5_database:
                rospy.loginfo('\t\tnew depth dataset')
                dataset = hdf5_database.create_dataset(vid_topic,
                                                       (INITIAL_SIZE, topic_meta_info.height,
                                                        topic_meta_info.width),
                                                       maxshape=(
                                                           None, topic_meta_info.height,
                                                           topic_meta_info.width),
                                                       dtype=np.uint16,
                                                       chunks=(
                                                           CHUNK_SIZE,
                                                           topic_meta_info.height,
                                                           topic_meta_info.width))
            elif 'color' in vid_topic and vid_topic not in hdf5_database:
                rospy.loginfo('\t\tnew color dataset')
                dataset = hdf5_database.create_dataset(vid_topic,
                                                       (INITIAL_SIZE, topic_meta_info.height,
                                                        topic_meta_info.width, 3),
                                                       maxshape=(
                                                           None, topic_meta_info.height,
                                                           topic_meta_info.width, 3),
                                                       dtype=np.uint8,
                                                       chunks=(
                                                           CHUNK_SIZE,
                                                           topic_meta_info.height,
                                                           topic_meta_info.width,
                                                           3))
            else:
                rospy.loginfo(
                    '\t\tDataset already existed!! no action taken')
                continue

            hdf5_database.create_dataset(vid_topic_time, (INITIAL_SIZE,),
                                         maxshape=(None,),
                                         dtype=np.double, chunks=(CHUNK_SIZE,))

            dataset.attrs.create('height', topic_meta_info.height)
            dataset.attrs.create('width', topic_meta_info.width)
            dataset.attrs.create('D', topic_meta_info.D)
            dataset.attrs.create('K', topic_meta_info.K)
            dataset.attrs.create('R', topic_meta_info.R)
            dataset.attrs.create('P', topic_meta_info.P)
            dataset.attrs.create('binning_x', topic_meta_info.binning_x)
            dataset.attrs.create('binning_y', topic_meta_info.binning_y)
            cam_name = vid_topic.split('/')[-1]
            if cam_name in extrinsics:
                rospy.loginfo('Added Extrinsics')
                dataset.attrs.create(
                    'depth_to_color-rotation', extrinsics[cam_name]['rotation'])
                dataset.attrs.create(
                    'depth_to_color-translation', extrinsics[cam_name]['translation'])
        else:
            rospy.loginfo('\t\tNo meta info')
    return hdf5_database


def fill_mappings(color_timestamps, depth_timestamps, mapping_dataset, color_idx):
    """Fill in the mappings from color to depth times, starting at specified index

    The mapping will end up with an index to index coorespondence for the color
    data and a value to index coorespondence for the depth data. Ie, the x index
    in the mapping dataset cooresponds to the x index in the color dataset. For
    any index x with stored value y, that index cooresponds to index y in the depth
    dataset

    Will back up by one from color_idx, just in case the last stored value should be
    changed given new data

    Args:
        color_timestamps: The array like of timestamps for the color images
        depth_timestamps: The array like of timestamps for the depth images
        mapping_dataset: The array like of mappings from color to depth to edit
        color_idx: The index to start filling in the mapping dataset at.
    """
    if color_idx > 0:
        color_idx -= 1
        depth_idx = mapping_dataset[color_idx]
    else:
        depth_idx = 1
    color_timestamps = color_timestamps.flatten()
    depth_timestamps = depth_timestamps.flatten()
    if len(color_timestamps) == 0 or len(depth_timestamps) == 0:
        return
    if len(depth_timestamps) == 1:
        mapping_dataset[...] = 0
    depth_len = len(depth_timestamps)
    while color_idx < len(color_timestamps):
        color_timestamp = color_timestamps[color_idx]
        while color_timestamp >= depth_timestamps[depth_idx] and depth_idx < depth_len-1:
            depth_idx += 1
        prev_diff = abs(color_timestamp - depth_timestamps[depth_idx - 1])
        curr_diff = abs(color_timestamp - depth_timestamps[depth_idx])
        if prev_diff < curr_diff:
            mapping_dataset[color_idx] = depth_idx-1
        else:
            mapping_dataset[color_idx] = depth_idx

        color_idx += 1


def add_realsense_video_data(bag_file, hdf5_file, meta_data):
    """Add video data to the hdf5 file.

    For every video topic in the meta data, all of the images
    in the bag file will be added to the appropriate dataset
    in the hdf5 file, increasing size as needed.

    Note: Assumes that the hdf5 file already exists with the
          appropriate datasets already created

    Args:
        hdf5_file: The HDF5 file to add to
        bag_file: The bag file to extract data from
        meta_data: The meta data with mappings from the name
                   to store in the hdf5 file to the raw names
                   used in the bag files
    """
    data_info_mapping = match_video_2_data(meta_data)
    add_video_topics(hdf5_file,
                     data_info_mapping,
                     meta_data,
                     get_topic_info(data_info_mapping, bag_file, meta_data),
                     get_realsense_extrinsics(bag_file))
    bridge = CvBridge()
    outer_progress_bar = tqdm(data_info_mapping)

    for root_topic in outer_progress_bar:
        vid_topic = root_topic+'/data'
        raw_topic = meta_data['bag-mapping'][vid_topic]
        outer_progress_bar.set_description('Working on: {}'.format(vid_topic))

        with tqdm(total=bag_file.get_message_count(raw_topic)) as inner_progress_bar:
            for _, msg, _ in bag_file.read_messages([raw_topic]):

                dataset = hdf5_file[vid_topic]
                timestamp = hdf5_file[root_topic + '/time']

                dataset.resize(dataset.shape[0]+1, axis=0)
                timestamp.resize(timestamp.shape[0]+1, axis=0)

                if 'depth' in vid_topic:
                    img = bridge.imgmsg_to_cv2(msg, '16UC1')
                    dataset[dataset.shape[0]-1, :,
                            :] = img
                else:
                    img = bridge.imgmsg_to_cv2(msg, 'bgr8')
                    dataset[dataset.shape[0]-1, :, :,
                            :] = img

                timestamp[timestamp.shape[0] -
                          1] = msg.header.stamp.to_sec()
                inner_progress_bar.update(1)
    rospy.loginfo('matching depth')
    match_depth(hdf5_file, data_info_mapping)


def match_depth(hdf5_file, data_info_mapping):  # pylint: disable=too-many-branches
    """Match the depth to the color topics for videos.

    Adds a data structure to the hdf5 file objects which are passed in (modifying the
    existing objects).

    Args:
        hdf5_files: the list of hdf file objects to work on
        data_info_mapping: The mapping between video data topics and info data topics
    """
    color_topics = filter(lambda x: 'color' in x, data_info_mapping)
    for color_root_topic in color_topics:
        color_match_topic = color_root_topic+'/matched_depth_index'

        timestamps = {}

        if color_match_topic not in hdf5_file:
            match_dataset = hdf5_file.create_dataset(color_match_topic, (INITIAL_SIZE,),
                                                     maxshape=(None,),
                                                     dtype=np.uint32, chunks=(CHUNK_SIZE,))
        else:
            match_dataset = hdf5_file[color_match_topic]
        depth_root_topic = color_root_topic.replace('color', 'depth')
        color_time_topic = color_root_topic+'/time'
        depth_time_topic = depth_root_topic + '/time'

        if color_time_topic not in hdf5_file or depth_time_topic not in hdf5_file:
            continue

        for source, time_topic in zip(('color', 'depth'), (color_time_topic, depth_time_topic)):
            timestamp = hdf5_file[time_topic]

            if timestamp.shape[0] == 0:
                tstamp = None
            else:
                tstamp = np.expand_dims(np.asarray(timestamp), 0)

            timestamps[source] = copy.deepcopy(tstamp)
        if timestamps['color'] is None:
            rospy.loginfo(
                'No Timestamps in current HDF5 database for %s', color_time_topic)
            continue
        if timestamps['depth'] is None:
            rospy.loginfo(
                'No Timestamps in current HDF5 database for %s', depth_time_topic)
            continue

        start = len(match_dataset)
        match_dataset.resize(timestamps['color'].shape[1], axis=0)
        fill_mappings(timestamps['color'],
                      timestamps['depth'], match_dataset, start)


def add_dataset(hdf5_file, topic_name, dtype):
    """Add datasets for both the data and time to the hdf5 file.

    Simply create datasets at <topic_name>/data and
    <topic_name>/time

    Assumes that the data is stored 1 dimmensionally

    It is safe to call this if the datasets already exist, will
    do nothing if they already exist.

    Stores time as a 64 bit double. It is expected that time is being
    stored as seconds since epoch, giving 10^-5 sec accuracy

    Args:
        hdf5_file: The hdf5 file to add to
        topic_name: The root topic name to work with
        dtype: The datatype for the data
    """
    if topic_name not in hdf5_file:
        # note: we are ignoring the hardware_id and values
        hdf5_file.create_dataset(topic_name+'/data', (0,),
                                 maxshape=(None,),
                                 dtype=dtype)
    if topic_name+'/time' not in hdf5_file:
        hdf5_file.create_dataset(topic_name+'/time', (0,),
                                 maxshape=(None,),
                                 dtype=np.double)


def add_to_dataset(topic_name, hdf5_file, data, time):
    """Add data to a dataset.

    Will resize the datasets to get longer by one. Adding both
    the data and time

    Args:
        topic_name: The root topic name (will edit root/time and root/data)
        hdf5_file: The hdf5 file to work with
        data: The data to add
        time:
    """
    dataset = hdf5_file[topic_name+'/data']
    dataset.resize(dataset.shape[0]+1, axis=0)
    dataset[dataset.shape[0]-1] = data
    dataset_time = hdf5_file[topic_name+'/time']
    dataset_time.resize(dataset_time.shape[0]+1, axis=0)
    dataset_time[dataset_time.shape[0]-1] = time


def add_topic_client_count(bag_file, hdf5_file):
    """Add the client_count topic from the bag_file

    Args:
        bag_file: The bag file to read from
        hdf5_file: The HDF5 file to write to
    """
    rospy.loginfo('adding client count')
    topic_name = '/client_count'
    add_dataset(hdf5_file, topic_name, np.short)
    for _, msg, msg_time in bag_file.read_messages([topic_name]):
        add_to_dataset(topic_name, hdf5_file, msg.data, msg_time.to_sec())


def add_topic_diagnostics(bag_file, hdf5_file):
    """Add the /diagnostics and /diagnostics_agg topics to the
    hdf5 file

    Args:
        bag_file: The bag file to read from
        hdf5_file: The HDF5 file to write to
    """
    rospy.loginfo('adding diagnostics')
    for topic_name in ('/diagnostics', '/diagnostics_agg'):
        topic_name = '/diagnostics'
        add_dataset(hdf5_file, topic_name, DIAGNOSTIC_DTYPE)
        for _, msg, _ in bag_file.read_messages([topic_name]):
            for status in msg.status:
                add_to_dataset(
                    topic_name,
                    hdf5_file,
                    (status.level, status.name, status.message),
                    msg.header.stamp.to_sec())


def add_topic_cpu(bag_file, hdf5_file):
    """Add the /cpu_stats topic to the hdf5 file

    Args:
        bag_file: The bag file to read from
        hdf5_file: The hdf5 file to write to
    """
    rospy.loginfo('adding cpu stats')
    topic_name = '/cpu_stats'
    mapped_name = '/stats/cpu'
    add_dataset(hdf5_file, mapped_name, float)
    for _, msg, msg_time in bag_file.read_messages([topic_name]):
        add_to_dataset(mapped_name, hdf5_file,
                       msg.percent_utilization, msg_time.to_sec())


def add_topic_diagnostics_toplevel(bag_file, hdf5_file):
    """Add the /diagnostics_toplevel_state topic to the hdf5 file

    Args:
        bag_file: The bag file to read from
        hdf5_file: The HDF5 file to write to
    """
    rospy.loginfo('adding diagnostics toplevel state')
    topic_name = '/diagnostics_toplevel_state'
    add_dataset(hdf5_file, topic_name, DIAGNOSTIC_DTYPE)
    for _, msg, msg_time in bag_file.read_messages([topic_name]):
        add_to_dataset(
            topic_name,
            hdf5_file,
            (msg.level, msg.name, msg.message),
            msg_time.to_sec())


def add_topic_face_state(bag_file, hdf5_file):
    """Add the /face_state topic to the HDF5 file

    Args:
        bag_file: The bag file to read from
        hdf5_file: The HDF5 file to write to
    """
    rospy.loginfo('adding face state')
    topic_name = '/face_state'
    # there are many more fields in the face state that could
    # be brought over
    add_dataset(hdf5_file, topic_name, FACE_DTYPE)
    for _, msg, msg_time in bag_file.read_messages([topic_name]):
        add_to_dataset(
            topic_name,
            hdf5_file,
            (msg.eye_name, msg.mouth_name, msg.mouth_description),
            msg_time.to_sec())


def add_topic_game_runner_actions(bag_file, hdf5_file):
    """Add the /game_runner_actions topic to the HDF5 file

    The data is stored in a compound data type with the speech
    as a string, the step id as an integer and the joint targets
    as a json object stored in a string. In order to use the joint
    targets, they will need to be de-serialized.

    Working with compound datatypes is a little wacky. To index,
    do something like:
        ```{python}
        dataset = hdf5_file['/game_runner_actions/data']
        speech = dataset[0]['speech']
        targets = json.loads[0]['targets']
        ```

    Args:
        bag_file: The bag file to read from
        hdf5_file: The HDF5 file to write to
    """
    rospy.loginfo('adding game actions')
    topic_name = '/game_runner_actions'
    mapped_name = '/game_runner/actions'
    add_dataset(hdf5_file, mapped_name, GAME_ACTION_DTYPE)
    for _, msg, msg_time in bag_file.read_messages([topic_name]):
        msg_dict = message_converter.convert_ros_message_to_dictionary(
            msg)

        add_to_dataset(
            mapped_name,
            hdf5_file,
            np.array(
                [(
                    msg.speech,
                    msg.step_id,
                    json.dumps(msg_dict['targets'])
                )],
                dtype=GAME_ACTION_DTYPE
            ),
            msg_time.to_sec()
        )


def add_topic_game_runner_command_opts(bag_file, hdf5_file):
    """Adds the topic game runner options

    The options are added in as a variable length array of strings

    Args:
        bag_file: the bag file to read from
        hdf5_file: the hdf5 file to write to
    """
    rospy.loginfo('adding game runner command options')
    topic_name = '/game_runner_command_opts'
    mapped_name = '/game_runner/command_opts'

    add_dataset(hdf5_file, mapped_name, h5py.vlen_dtype(h5py.string_dtype()))
    for _, msg, msg_time in bag_file.read_messages([topic_name]):
        add_to_dataset(mapped_name, hdf5_file, msg.options, msg_time.to_sec())


def add_topic_game_runner_commands(bag_file, hdf5_file):
    """Adds the topic game runner commands

    Args:
        bag_file: the bag file to read from
        hdf5_file: the hdf5 file to write to
    """
    rospy.loginfo('adding game runner commands')
    topic_name = '/game_runner_commands'
    mapped_name = '/game_runner/commands'

    add_dataset(hdf5_file, mapped_name, h5py.string_dtype())
    for _, msg, msg_time in bag_file.read_messages([topic_name]):
        add_to_dataset(mapped_name, hdf5_file, msg.command, msg_time.to_sec())


def add_topic_game_runner_def(bag_file, hdf5_file):
    """Adds the topic game runner definitions

    Note: the entire definition is just serialized as a json object

    Args:
        bag_file: the bag file to read from
        hdf5_file: the hdf5 file to write to
    """
    rospy.loginfo('adding game runner definitions')
    topic_name = '/game_runner_def'
    mapped_name = '/game_runner/def'

    add_dataset(hdf5_file, mapped_name, h5py.string_dtype())
    for _, msg, msg_time in bag_file.read_messages([topic_name]):
        add_to_dataset(
            mapped_name,
            hdf5_file,
            json_message_converter.convert_ros_message_to_json(msg),
            msg_time.to_sec()
        )


def add_topic_game_runner_state(bag_file, hdf5_file):
    """Adds the topic game runner state

    Args:
        bag_file: the bag file to read from
        hdf5_file: the hdf5 file to write to
    """
    rospy.loginfo('adding game runner state')
    topic_name = '/game_runner_state'
    mapped_name = '/game_runner/state'

    add_dataset(hdf5_file, mapped_name, h5py.string_dtype())
    for _, msg, msg_time in bag_file.read_messages([topic_name]):
        add_to_dataset(mapped_name, hdf5_file, msg.state, msg_time.to_sec())


def add_topic_game_runner_text(bag_file, hdf5_file):
    """Adds the topic game runner text

    This is the text that the game says. This works regardless of
    whether the robot is speaking or not.

    Args:
        bag_file: the bag file to read from
        hdf5_file: the hdf5 file to write to
    """
    rospy.loginfo('adding game runner text')
    topic_name = '/game_runner_text'
    mapped_name = '/game_runner/text'

    add_dataset(hdf5_file, mapped_name, h5py.string_dtype())
    for _, msg, msg_time in bag_file.read_messages([topic_name]):
        add_to_dataset(mapped_name, hdf5_file, msg.data, msg_time.to_sec())


def add_topic_hdd_stats(bag_file, hdf5_file):
    """Adds the topic hdd stats

    Args:
        bag_file: the bag file to read from
        hdf5_file: the hdf5 file to write to
    """
    rospy.loginfo('adding hdd stats')
    topic_name = '/hdd_stats'
    mapped_name = '/stats/hdd'

    add_dataset(hdf5_file, mapped_name, float)
    for _, msg, msg_time in bag_file.read_messages([topic_name]):
        add_to_dataset(mapped_name, hdf5_file,
                       msg.percent_free, msg_time.to_sec())


def add_topic_joint_states(bag_file, hdf5_file):
    """Adds the topic joint states

    Note: the entire message is just serialized as a json object

    Args:
        bag_file: the bag file to read from
        hdf5_file: the hdf5 file to write to
    """
    rospy.loginfo('adding joint states')
    topic_name = '/joint_states'

    add_dataset(hdf5_file, topic_name, h5py.string_dtype())
    for _, msg, _ in bag_file.read_messages([topic_name]):
        add_to_dataset(
            topic_name,
            hdf5_file,
            json_message_converter.convert_ros_message_to_json(msg),
            msg.header.stamp.to_sec()
        )


def add_topic_mem_stats(bag_file, hdf5_file):
    """Adds the topic mem stats

    Args:
        bag_file: the bag file to read from
        hdf5_file: the hdf5 file to write to
    """
    rospy.loginfo('adding mem stats')
    topic_name = '/mem_stats'
    mapped_name = '/stats/mem'

    add_dataset(hdf5_file, mapped_name, float)
    for _, msg, msg_time in bag_file.read_messages([topic_name]):
        add_to_dataset(mapped_name, hdf5_file,
                       msg.percent_used, msg_time.to_sec())


def add_topic_net_stats(bag_file, hdf5_file):
    """Adds the topic net stats

    Args:
        bag_file: the bag file to read from
        hdf5_file: the hdf5 file to write to
    """
    rospy.loginfo('adding net stats')
    topic_name = '/net_stats'
    mapped_name = '/stats/net'

    add_dataset(hdf5_file, mapped_name, NET_DTYPE)
    for _, msg, msg_time in bag_file.read_messages([topic_name]):
        add_to_dataset(mapped_name, hdf5_file,
                       (msg.link_quality, msg.signal_strength,
                           msg.network_ssid, msg.ip_addr),
                       msg_time.to_sec())


def add_topic_motor_commands(bag_file, hdf5_file):
    """Adds the topic game runner text

    This is the text that commands the motor to stop or relax.

    Args:
        bag_file: the bag file to read from
        hdf5_file: the hdf5 file to write to
    """
    rospy.loginfo('adding game motor commands')
    topic_name = '/motor_commands'

    add_dataset(hdf5_file, topic_name, h5py.string_dtype())
    for _, msg, msg_time in bag_file.read_messages([topic_name]):
        add_to_dataset(topic_name, hdf5_file, msg.data, msg_time.to_sec())


def add_topic_move_goal(bag_file, hdf5_file):
    """Adds the topic move/goal

    Note: the entire goal (not entire message) is just serialized as a json object

    Args:
        bag_file: the bag file to read from
        hdf5_file: the hdf5 file to write to
    """
    rospy.loginfo('adding move goal')
    topic_name = '/move/goal'

    add_dataset(hdf5_file, topic_name, h5py.string_dtype())
    for _, msg, _ in bag_file.read_messages([topic_name]):
        msg_dict = message_converter.convert_ros_message_to_dictionary(
            msg)
        add_to_dataset(
            topic_name,
            hdf5_file,
            json.dumps(msg_dict['goal']),
            msg.header.stamp.to_sec()
        )


def add_topic_move_result(bag_file, hdf5_file):
    """Adds the topic move/result

    Args:
        bag_file: the bag file to read from
        hdf5_file: the hdf5 file to write to
    """
    rospy.loginfo('adding move result')
    topic_name = '/move/result'

    add_dataset(hdf5_file, topic_name, MOVE_RESULT_DTYPE)
    for _, msg, _ in bag_file.read_messages([topic_name]):
        add_to_dataset(
            topic_name,
            hdf5_file,
            (msg.result.completed, msg.result.positional_error,
             msg.status.status, msg.status.text),
            msg.header.stamp.to_sec()
        )


def add_topic_move_feedback(bag_file, hdf5_file):
    """Adds the topic move/feedback

    Args:
        bag_file: the bag file to read from
        hdf5_file: the hdf5 file to write to
    """
    rospy.loginfo('adding move feedback')
    topic_name = '/move/feedback'

    add_dataset(hdf5_file, topic_name, MOVE_FEEDBACK_DTYPE)
    for _, msg, _ in bag_file.read_messages([topic_name]):
        add_to_dataset(
            topic_name,
            hdf5_file,
            (msg.feedback.time_elapsed, msg.feeback.time_remaining,
             msg.feedback.move_number),
            msg.header.stamp.to_sec()
        )


def add_topic_rosout(bag_file, hdf5_file):
    """Adds the topic rosout

    Args:
        bag_file: the bag file to read from
        hdf5_file: the hdf5 file to write to
    """
    for topic_name in ('/rosout', '/rosout_agg'):
        rospy.loginfo('adding %s', topic_name)

        add_dataset(hdf5_file, topic_name, ROSOUT_DTYPE)
        for _, msg, _ in bag_file.read_messages([topic_name]):
            add_to_dataset(
                topic_name,
                hdf5_file,
                (
                    msg.level,
                    msg.name,
                    msg.msg,
                    msg.file,
                    msg.function,
                    msg.line
                ),
                msg.header.stamp.to_sec()
            )


def add_topic_move_status(bag_file, hdf5_file):
    """Adds the topic move/status

    Note: this isn't the best way to get status, it is
    kind of a messy format. Not clear what it is storing.

    Args:
        bag_file: the bag file to read from
        hdf5_file: the hdf5 file to write to
    """
    rospy.loginfo('adding move status')
    topic_name = '/move/status'

    add_dataset(hdf5_file, topic_name, h5py.vlen_dtype(h5py.string_dtype()))
    for _, msg, _ in bag_file.read_messages([topic_name]):
        add_to_dataset(
            topic_name,
            hdf5_file,
            np.asarray(msg.status_list, dtype=np.dtype('U')),
            msg.header.stamp.to_sec()
        )


def add_topic_move_available(bag_file, hdf5_file):
    """Adds the topic move_available

    Note: this isn't the best way to get status, it is
    kind of a messy format. Not clear what it is storing.

    Args:
        bag_file: the bag file to read from
        hdf5_file: the hdf5 file to write to
    """
    rospy.loginfo('adding move_available')
    topic_name = '/move_available'
    mapped_name = '/move/available'

    add_dataset(hdf5_file, mapped_name, bool)
    for _, msg, msg_time in bag_file.read_messages([topic_name]):
        add_to_dataset(
            mapped_name,
            hdf5_file,
            msg.data,
            msg_time.to_sec()
        )


def add_topic_robot_audio(bag_file, hdf5_file):
    """Adds the topic robot_audio/audio_relay

    Note: this isn't the best way to get status, it is
    kind of a messy format. Not clear what it is storing.

    Args:
        bag_file: the bag file to read from
        hdf5_file: the hdf5 file to write to
    """
    rospy.loginfo('adding robot audio')
    topic_name = '/robot_audio/audio_relay'

    add_dataset(hdf5_file, topic_name, h5py.vlen_dtype(np.uint8))
    for _, msg, msg_time in bag_file.read_messages([topic_name]):
        data = []
        data += msg.data
        add_to_dataset(
            topic_name,
            hdf5_file,
            data,
            msg_time.to_sec()
        )


def add_topic_sound_cancel(bag_file, hdf5_file):
    """Adds the sound cancel topic

    Args:
        bag_file: the bag file to read from
        hdf5_file: the hdf5 file to write to
    """
    topic_name = '/sound_play/cancel'
    rospy.loginfo('adding %s', topic_name)

    add_dataset(hdf5_file, topic_name, h5py.string_dtype())
    for _, msg, msg_time in bag_file.read_messages([topic_name]):
        add_to_dataset(
            topic_name,
            hdf5_file,
            msg.id,
            msg_time.to_sec()
        )

#     /sound_play/feedback:    sound_play/SoundRequestActionFeedback


def add_topic_sound_feedback(bag_file, hdf5_file):
    """Adds the topic soundplay feedback

    Args:
        bag_file: the bag file to read from
        hdf5_file: the hdf5 file to write to
    """
    topic_name = '/sound_play/feedback'
    rospy.loginfo('adding %s', topic_name)

    add_dataset(hdf5_file, topic_name, SOUND_FEEDBACK_DTYPE)
    for _, msg, _ in bag_file.read_messages([topic_name]):
        add_to_dataset(
            topic_name,
            hdf5_file,
            (
                msg.feedback.playing,
                msg.feedback.stamp.to_sec()
            ),
            msg.header.stamp.to_sec()
        )


#     /sound_play/goal:    sound_play/SoundRequestActionGoal
def add_topic_sound_goal(bag_file, hdf5_file):
    """Adds the topic soundplay goal

    Args:
        bag_file: the bag file to read from
        hdf5_file: the hdf5 file to write to
    """
    topic_name = '/sound_play/goal'
    rospy.loginfo('adding %s', topic_name)

    add_dataset(hdf5_file, topic_name, SOUND_GOAL_DTYPE)
    for _, msg, _ in bag_file.read_messages([topic_name]):
        add_to_dataset(
            topic_name,
            hdf5_file,
            (
                msg.goal.sound_request.command,
                msg.goal.sound_request.volume,
                msg.goal.sound_request.arg,
                msg.goal.sound_request.arg2
            ),
            msg.header.stamp.to_sec()
        )

#     /sound_play/result:  sound_play/SoundRequestActionResult


def add_topic_sound_result(bag_file, hdf5_file):
    """Adds the topic soundplay result

    Args:
        bag_file: the bag file to read from
        hdf5_file: the hdf5 file to write to
    """
    topic_name = '/sound_play/result'
    rospy.loginfo('adding %s', topic_name)

    add_dataset(hdf5_file, topic_name, SOUND_FEEDBACK_DTYPE)
    for _, msg, _ in bag_file.read_messages([topic_name]):
        add_to_dataset(
            topic_name,
            hdf5_file,
            (
                msg.result.playing,
                msg.result.stamp.to_sec()
            ),
            msg.header.stamp.to_sec()
        )


def add_topic_sound_status(bag_file, hdf5_file):
    """Adds the topic sound_play/status

    Note: this isn't the best way to get status, it is
    kind of a messy format. Not clear what it is storing.

    Args:
        bag_file: the bag file to read from
        hdf5_file: the hdf5 file to write to
    """
    rospy.loginfo('adding sound play status')
    topic_name = '/sound_play/status'

    add_dataset(hdf5_file, topic_name, h5py.vlen_dtype(h5py.string_dtype()))
    for _, msg, _ in bag_file.read_messages([topic_name]):
        add_to_dataset(
            topic_name,
            hdf5_file,
            np.asarray(msg.status_list, dtype=np.dtype('U')),
            msg.header.stamp.to_sec()
        )


def add_topic_tf(bag_file, hdf5_file):
    """Adds the tf topic and tf_static topic

    This expresses a transform from coordinate frame parent_frame_id
    to the coordinate frame child_frame_id

    Args:
        bag_file: the bag file to read from
        hdf5_file: the hdf5 file to write to
    """
    for topic_name in ('/tf', '/tf_static'):
        rospy.loginfo('adding %s', topic_name)

        add_dataset(hdf5_file, topic_name, TRANSFORM_STAMPED_DTYPE)
        for _, msg, _ in bag_file.read_messages([topic_name]):
            for transform_msg in msg.transforms:
                rotation = np.asarray(
                    (
                        transform_msg.transform.rotation.x,
                        transform_msg.transform.rotation.y,
                        transform_msg.transform.rotation.z,
                        transform_msg.transform.rotation.w
                    ), dtype=QUATERNION_DTYPE)
                translation = np.asarray(
                    (
                        transform_msg.transform.translation.x,
                        transform_msg.transform.translation.y,
                        transform_msg.transform.translation.z
                    ), dtype=VECTOR3_DTYPE)
                transform = np.asarray(
                    (translation, rotation), dtype=TRANSFORM_DTYPE)

                add_to_dataset(
                    topic_name,
                    hdf5_file,
                    (
                        transform_msg.child_frame_id,
                        transform,
                        transform_msg.header.frame_id
                    ),
                    transform_msg.header.stamp.to_sec()
                )


def add_realsense_extrinsics(bag_file, hdf5_file):
    """Adds the realsense extrinsics topics (upper and lower)

    Args:
        bag_file: the bag file to read from
        hdf5_file: the hdf5 file to write to
    """
    for topic_name, mapped_name in zip((
            '/upper_realsense/extrinsics/depth_to_color',
            '/lower_realsense/extrinsics/depth_to_color'
    ), (
        '/vid/upper/depth_to_color',
        '/vid/lower/depth_to_color'
    )):
        rospy.loginfo('adding %s', topic_name)

        add_dataset(hdf5_file, mapped_name, REALSENSE_EXTRINSICS_DTYPE)
        for _, msg, msg_time in bag_file.read_messages([topic_name]):
            add_to_dataset(
                mapped_name,
                hdf5_file,
                (
                    msg.rotation,
                    msg.translation
                ),
                msg_time.to_sec()
            )


def add_topic_tts_cancel(bag_file, hdf5_file):
    """Adds the topic tts/cancel

    Args:
        bag_file: the bag file to read from
        hdf5_file: the hdf5 file to write to
    """
    rospy.loginfo('adding tts cancel')
    topic_name = '/tts/cancel'

    add_dataset(hdf5_file, topic_name, h5py.string_dtype())
    for _, msg, msg_time in bag_file.read_messages([topic_name]):
        add_to_dataset(
            topic_name,
            hdf5_file,
            msg.id,
            msg_time.to_sec()
        )


def add_topic_tts_feedback(bag_file, hdf5_file):
    """Adds the topic tts/feedback

    Args:
        bag_file: the bag file to read from
        hdf5_file: the hdf5 file to write to
    """
    rospy.loginfo('adding tts feedback')
    topic_name = '/tts/feedback'

    add_dataset(hdf5_file, topic_name, h5py.string_dtype())
    for _, msg, _ in bag_file.read_messages([topic_name]):
        add_to_dataset(
            topic_name,
            hdf5_file,
            msg.feedback.data,
            msg.header.stamp.to_sec()
        )


def add_topic_tts_goal(bag_file, hdf5_file):
    """Adds the topic tts goal

    Args:
        bag_file: the bag file to read from
        hdf5_file: the hdf5 file to write to
    """
    topic_name = '/tts/goal'
    rospy.loginfo('adding %s', topic_name)

    add_dataset(hdf5_file, topic_name, SPEECH_GOAL_DTYPE)
    for _, msg, _ in bag_file.read_messages([topic_name]):
        add_to_dataset(
            topic_name,
            hdf5_file,
            (
                msg.goal.text,
                msg.goal.metadata
            ),
            msg.header.stamp.to_sec()
        )


def add_topic_tts_result(bag_file, hdf5_file):
    """Adds the topic tts/result

    Args:
        bag_file: the bag file to read from
        hdf5_file: the hdf5 file to write to
    """
    topic_name = '/tts/result'
    rospy.loginfo('adding %s', topic_name)

    add_dataset(hdf5_file, topic_name, h5py.string_dtype())
    for _, msg, _ in bag_file.read_messages([topic_name]):
        add_to_dataset(
            topic_name,
            hdf5_file,
            msg.result.response,
            msg.header.stamp.to_sec()
        )


def add_topic_tts_status(bag_file, hdf5_file):
    """Adds the topic tts/status

    Note: this isn't the best way to get status, it is
    kind of a messy format. Not clear what it is storing.

    Args:
        bag_file: the bag file to read from
        hdf5_file: the hdf5 file to write to
    """
    topic_name = '/tts/status'
    rospy.loginfo('adding %s', topic_name)

    add_dataset(hdf5_file, topic_name, h5py.vlen_dtype(h5py.string_dtype()))
    for _, msg, _ in bag_file.read_messages([topic_name]):
        add_to_dataset(
            topic_name,
            hdf5_file,
            np.asarray(msg.status_list, dtype=np.dtype('U')),
            msg.header.stamp.to_sec()
        )


def add_topic_tts_state(bag_file, hdf5_file):
    """Adds the topic tts/status

    Note: this isn't the best way to get status, it is
    kind of a messy format. Not clear what it is storing.

    Args:
        bag_file: the bag file to read from
        hdf5_file: the hdf5 file to write to
    """
    topic_name = '/tts_state'
    remapped_name = '/tts/state'
    rospy.loginfo('adding %s', topic_name)

    add_dataset(hdf5_file, remapped_name, TTS_STATE_DTYPE)
    for _, msg, msg_time in bag_file.read_messages([topic_name]):
        add_to_dataset(
            remapped_name,
            hdf5_file,
            (msg.state, msg.text),
            msg_time.to_sec()
        )


def add_topic_tts_utterances(bag_file, hdf5_file):
    """Adds the topic tts/utterances

    Args:
        bag_file: the bag file to read from
        hdf5_file: the hdf5 file to write to
    """
    topic_name = '/tts_utterances'
    remapped_name = '/tts/utterances'
    rospy.loginfo('adding %s', topic_name)

    add_dataset(hdf5_file, remapped_name, h5py.string_dtype())
    for _, msg, msg_time in bag_file.read_messages([topic_name]):
        add_to_dataset(
            remapped_name,
            hdf5_file,
            msg.text,
            msg_time.to_sec()
        )


def add_topic_mobile_base_velocity(bag_file, hdf5_file):
    """Adds the topic /mobile_base/commands/velocity

    Args:
        bag_file: the bag file to read from
        hdf5_file: the hdf5 file to write to
    """
    topic_name = '/mobile_base/commands/velocity'
    rospy.loginfo('adding %s', topic_name)

    add_dataset(hdf5_file, topic_name, TWIST_DTYPE)
    for _, msg, msg_time in bag_file.read_messages([topic_name]):
        add_to_dataset(
            topic_name,
            hdf5_file,
            (
                np.asarray((
                    msg.linear.x,
                    msg.linear.y,
                    msg.linear.z
                ), dtype=VECTOR3_DTYPE),
                np.asarray((
                    msg.angular.x,
                    msg.angular.y,
                    msg.angular.z
                ), dtype=VECTOR3_DTYPE)
            ),
            msg_time.to_sec()
        )


def add_topic_mobile_base_imu(bag_file, hdf5_file):
    """Adds the topic /mobile_base/sensors/imu_data

    Args:
        bag_file: the bag file to read from
        hdf5_file: the hdf5 file to write to
    """
    topic_name = '/mobile_base/sensors/imu_data'
    rospy.loginfo('adding %s', topic_name)

    add_dataset(hdf5_file, topic_name, IMU_DTYPE)
    for _, msg, msg_time in bag_file.read_messages([topic_name]):
        add_to_dataset(
            topic_name,
            hdf5_file,
            (
                np.asarray((
                    msg.orientation.x,
                    msg.orientation.y,
                    msg.orientation.z,
                    msg.orientation.w,
                ), dtype=QUATERNION_DTYPE),
                msg.orientation_covariance,
                np.asarray((
                    msg.angular_velocity.x,
                    msg.angular_velocity.y,
                    msg.angular_velocity.z
                ), dtype=VECTOR3_DTYPE),
                msg.angular_velocity_covariance,
                np.asarray((
                    msg.linear_acceleration.x,
                    msg.linear_acceleration.y,
                    msg.linear_acceleration.z
                ), dtype=VECTOR3_DTYPE),
                msg.linear_acceleration_covariance
            ),
            msg_time.to_sec()
        )


def add_topic_odom(bag_file, hdf5_file):
    """Adds the topic /odom

    Args:
        bag_file: the bag file to read from
        hdf5_file: the hdf5 file to write to
    """
    topic_name = '/odom'
    rospy.loginfo('adding %s', topic_name)

    add_dataset(hdf5_file, topic_name, ODOMETRY_DTYPE)
    for _, msg, _ in bag_file.read_messages([topic_name]):
        add_to_dataset(
            topic_name,
            hdf5_file,
            (
                msg.child_frame_id,
                np.asarray((
                    np.asarray((
                        np.asarray((
                            msg.pose.pose.position.x,
                            msg.pose.pose.position.y,
                            msg.pose.pose.position.z
                        ), dtype=VECTOR3_DTYPE),
                        np.asarray((
                            msg.pose.pose.orientation.x,
                            msg.pose.pose.orientation.y,
                            msg.pose.pose.orientation.z,
                            msg.pose.pose.orientation.w
                        ), dtype=QUATERNION_DTYPE)),
                        dtype=TRANSFORM_DTYPE),
                    msg.pose.covariance
                ), dtype=POSE_COVARIANCE_DTYPE),
                np.asarray((
                    np.asarray((
                        np.asarray((
                            msg.twist.twist.linear.x,
                            msg.twist.twist.linear.y,
                            msg.twist.twist.linear.z
                        ), dtype=VECTOR3_DTYPE),
                        np.asarray((
                            msg.twist.twist.angular.x,
                            msg.twist.twist.angular.y,
                            msg.twist.twist.angular.z
                        ), dtype=VECTOR3_DTYPE)
                    ), dtype=TWIST_DTYPE),
                    msg.twist.covariance
                ), dtype=TWIST_COVARIANCE_DTYPE),
                msg.header.frame_id
            ),
            msg.header.stamp.to_sec()
        )


def add_diagnostic_topics(bag_file, hdf5_file):
    """add the diagnostic topics to the hdf file

    Args:
        bag_file: bag file to read from
        hdf5_file: hdf5 file to write to
    """
    add_topic_diagnostics(bag_file, hdf5_file)
    add_topic_diagnostics_toplevel(bag_file, hdf5_file)
    add_topic_rosout(bag_file, hdf5_file)


def add_system_stats_topics(bag_file, hdf5_file):
    """add the system stats topics to the hdf file

    Args:
        bag_file: bag file to read from
        hdf5_file: hdf5 file to write to
    """
    add_topic_cpu(bag_file, hdf5_file)
    add_topic_hdd_stats(bag_file, hdf5_file)
    add_topic_mem_stats(bag_file, hdf5_file)
    add_topic_net_stats(bag_file, hdf5_file)


def add_game_topics(bag_file, hdf5_file):
    """add the game runner topics to the hdf file

    Args:
        bag_file: bag file to read from
        hdf5_file: hdf5 file to write to
    """
    add_topic_game_runner_actions(bag_file, hdf5_file)
    add_topic_game_runner_command_opts(bag_file, hdf5_file)
    add_topic_game_runner_commands(bag_file, hdf5_file)
    add_topic_game_runner_def(bag_file, hdf5_file)
    add_topic_game_runner_state(bag_file, hdf5_file)
    add_topic_game_runner_text(bag_file, hdf5_file)


def add_move_topics(bag_file, hdf5_file):
    """add the arm movement topics to the hdf file

    Args:
        bag_file: bag file to read from
        hdf5_file: hdf5 file to write to
    """
    add_topic_move_goal(bag_file, hdf5_file)
    add_topic_move_result(bag_file, hdf5_file)
    add_topic_move_status(bag_file, hdf5_file)
    add_topic_move_feedback(bag_file, hdf5_file)
    add_topic_move_available(bag_file, hdf5_file)


def add_sound_topics(bag_file, hdf5_file):
    """add the sound topics to the hdf file

    Args:
        bag_file: bag file to read from
        hdf5_file: hdf5 file to write to
    """
    add_topic_sound_cancel(bag_file, hdf5_file)
    add_topic_sound_feedback(bag_file, hdf5_file)
    add_topic_sound_goal(bag_file, hdf5_file)
    add_topic_sound_result(bag_file, hdf5_file)
    add_topic_sound_status(bag_file, hdf5_file)


def add_tts_topics(bag_file, hdf5_file):
    """add the tts topics to the hdf file

    Args:
        bag_file: bag file to read from
        hdf5_file: hdf5 file to write to
    """
    add_topic_tts_cancel(bag_file, hdf5_file)
    add_topic_tts_feedback(bag_file, hdf5_file)
    add_topic_tts_goal(bag_file, hdf5_file)
    add_topic_tts_result(bag_file, hdf5_file)
    add_topic_tts_status(bag_file, hdf5_file)
    add_topic_tts_state(bag_file, hdf5_file)
    add_topic_tts_utterances(bag_file, hdf5_file)


def add_mobile_base_topics(bag_file, hdf5_file):
    """add the mobile base topics to the hdf file

    Args:
        bag_file: bag file to read from
        hdf5_file: hdf5 file to write to
    """
    add_topic_mobile_base_velocity(bag_file, hdf5_file)
    add_topic_mobile_base_imu(bag_file, hdf5_file)


def add_realsense_topics(bag_file, hdf5_file):
    """add the realsense topics to the hdf file

    Args:
        bag_file: bag file to read from
        hdf5_file: hdf5 file to write to
    """
    meta_data = {
        'bag-mapping': {
            'vid/upper/color/data': '/upper_realsense/color/image_raw_relay',
            'vid/upper/color/info': '/upper_realsense/color/camera_info',
            'vid/upper/depth/data': '/upper_realsense/depth/image_rect_raw_relay',
            'vid/upper/depth/info': '/upper_realsense/depth/camera_info',
            'vid/lower/color/data': '/lower_realsense/color/image_raw_relay',
            'vid/lower/color/info': '/lower_realsense/color/camera_info',
            'vid/lower/depth/data': '/lower_realsense/depth/image_rect_raw_relay',
            'vid/lower/depth/info': '/lower_realsense/depth/camera_info'
        }}
    add_realsense_video_data(bag_file, hdf5_file, meta_data)
    add_realsense_extrinsics(bag_file, hdf5_file)


def node():
    """The ROS node from which bag access, data parsing, and and hdf5 file generation
       is orchestrated
    """
    rospy.init_node('rosbag_to_hdf5_node')
    rospy.loginfo('started rosbag to hdf5 conversion')
    bag_filename = os.path.expanduser(rospy.get_param('~bag_file', None))
    rospy.loginfo('Bag file: %s', bag_filename)
    out_dir = os.path.expanduser(rospy.get_param('~out_dir', None))
    rospy.loginfo('output directory: %s', out_dir)

    bag_file = load_bag_file(bag_filename)
    hdf5_file = load_hdf_file(out_dir)

    add_realsense_topics(bag_file, hdf5_file)
    add_game_topics(bag_file, hdf5_file)
    add_system_stats_topics(bag_file, hdf5_file)
    add_diagnostic_topics(bag_file, hdf5_file)
    add_move_topics(bag_file, hdf5_file)
    add_sound_topics(bag_file, hdf5_file)
    add_tts_topics(bag_file, hdf5_file)
    add_mobile_base_topics(bag_file, hdf5_file)

    add_topic_client_count(bag_file, hdf5_file)
    add_topic_face_state(bag_file, hdf5_file)
    add_topic_joint_states(bag_file, hdf5_file)
    add_topic_motor_commands(bag_file, hdf5_file)
    add_topic_robot_audio(bag_file, hdf5_file)
    add_topic_tf(bag_file, hdf5_file)
    add_topic_odom(bag_file, hdf5_file)

    rospy.loginfo('closing hdf5 file')
    hdf5_file.close()

    rospy.loginfo('Exiting Process')
    rospy.signal_shutdown('Exiting Cleanly')


# Added:
#     /client_count:   std_msgs/Int32
#     /cpu_stats:  system_monitor/CPUutil
#     /diagnostics:    diagnostic_msgs/DiagnosticArray
#     /diagnostics_agg:    diagnostic_msgs/DiagnosticArray
#     /diagnostics_toplevel_state:     diagnostic_msgs/DiagnosticStatus
#     /face_state:     flo_face_defs/FaceState
#     /game_runner_actions:    flo_core_defs/GameAction
#     /game_runner_command_opts:   flo_core_defs/GameCommandOptions
#     /game_runner_commands:   flo_core_defs/GameCommand
#     /game_runner_def:    flo_core_defs/GameDef
#     /game_runner_state:  flo_core_defs/GameState
#     /game_runner_text:   std_msgs/String
#     /hdd_stats:  system_monitor/HDDutil
#     /joint_states:   sensor_msgs/JointState
#     /lower_realsense/color/image_raw_relay:  sensor_msgs/Image
#     /lower_realsense/depth/image_rect_raw_relay:     sensor_msgs/Image
#     /move/goal:  flo_humanoid_defs/MoveActionGoal
#     /move/result:    flo_humanoid_defs/MoveActionResult
#     /move/status:    actionlib_msgs/GoalStatusArray
#     /upper_realsense/color/image_raw_relay:  sensor_msgs/Image
#     /upper_realsense/depth/image_rect_raw_relay:     sensor_msgs/Image
#     /mem_stats:  system_monitor/MEMutil
#     /move/feedback:  flo_humanoid_defs/MoveActionFeedback
#     /move_available:     std_msgs/Bool
#     /net_stats:  system_monitor/NETstats
#     /robot_audio/audio_relay:    audio_common_msgs/AudioData
#     /rosout:     rosgraph_msgs/Log
#     /rosout_agg:     rosgraph_msgs/Log
#     /sound_play/cancel:  actionlib_msgs/GoalID
#     /sound_play/feedback:    sound_play/SoundRequestActionFeedback
#     /sound_play/goal:    sound_play/SoundRequestActionGoal
#     /sound_play/result:  sound_play/SoundRequestActionResult
#     /sound_play/status:  actionlib_msgs/GoalStatusArray
#     /tf:     tf2_msgs/TFMessage
#     /tf_static:  tf2_msgs/TFMessage
#     /lower_realsense/extrinsics/depth_to_color:  realsense2_camera/Extrinsics
#     /upper_realsense/extrinsics/depth_to_color:  realsense2_camera/Extrinsics
#     /tts/cancel:     actionlib_msgs/GoalID
#     /tts/feedback:   tts/SpeechActionFeedback
#     /tts/goal:   tts/SpeechActionGoal
#     /tts/result:     tts/SpeechActionResult
#     /tts/status:     actionlib_msgs/GoalStatusArray
#     /tts_state:  flo_core_defs/TTSState
#     /tts_utterances:     flo_core_defs/TTSUtterances
#     /odom:   nav_msgs/Odometry
#     /upper_realsense/color/camera_info:  sensor_msgs/CameraInfo
#     /upper_realsense/depth/camera_info:  sensor_msgs/CameraInfo
#     /lower_realsense/color/camera_info:  sensor_msgs/CameraInfo
#     /lower_realsense/depth/camera_info:  sensor_msgs/CameraInfo

# Not Added:
#     /cmd_vel_mux/active:     std_msgs/String
#     /cmd_vel_mux/keyboard_teleop:    geometry_msgs/Twist
#     /cmd_vel_mux/parameter_descriptions:     dynamic_reconfigure/ConfigDescription
#     /cmd_vel_mux/parameter_updates:  dynamic_reconfigure/Config
#     /cmd_vel_mux/safety_controller:  geometry_msgs/Twist
#     /connected_clients:  rosbridge_msgs/ConnectedClients
#     /keyop_vel_smoother/parameter_descriptions:  dynamic_reconfigure/ConfigDescription
#     /keyop_vel_smoother/parameter_updates:   dynamic_reconfigure/Config
#     /keyop_vel_smoother/raw_cmd_vel:     geometry_msgs/Twist
#     /mobile_base/controller_info:    kobuki_msgs/ControllerInfo
#     /mobile_base/debug/raw_control_command:  std_msgs/Int16MultiArray
#     /mobile_base/debug/raw_data_command:     std_msgs/String
#     /mobile_base/debug/raw_data_stream:  std_msgs/String
#     /mobile_base/events/bumper:  kobuki_msgs/BumperEvent
#     /mobile_base/events/cliff:   kobuki_msgs/CliffEvent
#     /mobile_base/events/robot_state:     kobuki_msgs/RobotStateEvent
#     /mobile_base/events/wheel_drop:  kobuki_msgs/WheelDropEvent
#     /mobile_base/sensors/core:   kobuki_msgs/SensorState
#     /mobile_base/sensors/dock_ir:    kobuki_msgs/DockInfraRed
#     /mobile_base/sensors/imu_data_raw:   sensor_msgs/Imu
#     /mobile_base/version_info:   kobuki_msgs/VersionInfo
#     /mobile_base_nodelet_manager/bond:   bond/Status
#     /remote_video_clean_relay:   sensor_msgs/Image


if __name__ == '__main__':
    node()
