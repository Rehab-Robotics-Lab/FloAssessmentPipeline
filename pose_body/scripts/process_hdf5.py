#!/usr/bin/env python3
"""Module to extract pose using openpose from hdf5 file"""

import argparse
import json
import numpy as np
import h5py
from pose_body.scripts.extract_poses import process_frames
from pose_body.scripts.extract_depth import add_stereo_depth
from tqdm import tqdm


def allkeys(obj):
    "Recursively find all datasets in an h5py.Group."
    keys = (obj.name,)
    if isinstance(obj, h5py.Group):
        for _, value in obj.items():
            if isinstance(value, h5py.Group):
                keys = keys + allkeys(value)
            else:
                keys = keys + (value.name,)
    return keys


def convert(video_pth, no_video_pth, transforms_pth, source, cam, rerun):
    # pylint: disable= too-many-statements
    # pylint: disable= too-many-arguments
    # pylint: disable= too-many-locals
    """Extract poses from video in hdf5 file and build new hdf
    file with poses, confidences, and other non-video data.

    Args:
        pth: the path of the hdf5 source file to work with, should also
             have a file named transforms.json located next to the hdf5
             file if the hdf5 file does not have the depth_to_color
             topics recorded.
        source: Where the file came from. Options are: 'mixed', 'podium',
                and 'robot'
    """

    # open hdf5 file
    try:
        hdf5_in = h5py.File(video_pth, 'r')
    except:  # pylint: disable=bare-except
        print('HDF5 database could not be read')
        raise
    print('opened hdf file for reading video')

    try:
        hdf5_out = h5py.File(no_video_pth, 'r+')
    except:  # pylint: disable=bare-except
        print('HDF5 database could not be read')
        raise
    print('opened hdf file for writing poses')

    transforms = None
    try:
        with open(transforms_pth, encoding='utf-8') as json_file:
            transforms = json.load(json_file)
        print('got transforms file')
    except:  # pylint: disable=bare-except
        print('cannot open transforms file')

    cam_root = f'/vid/{cam}'
    dset = f'{cam_root}/color/data'

    preexisting_keypoints = False
    if f'{cam_root}/openpose/keypoints' not in hdf5_out:
        keypoints_dset = hdf5_out.create_dataset(
            f'{cam_root}/openpose/keypoints', (hdf5_in[dset].len(), 25, 2), dtype=np.float32)
    else:
        keypoints_dset = hdf5_out[f'{cam_root}/openpose/keypoints']
        preexisting_keypoints = True
        print('keypoints already exist')

    preexisting_confidence = False
    if f'{cam_root}/openpose/confidence' not in hdf5_out:
        confidence_dset = hdf5_out.create_dataset(
            f'{cam_root}/openpose/confidence', (hdf5_in[dset].len(), 25), dtype=np.float32)
    else:
        confidence_dset = hdf5_out[f'{cam_root}/openpose/confidence']
        preexisting_confidence = True
        print('confidences already exist')

    if (not(preexisting_keypoints and preexisting_confidence)) or rerun:
        print('running pose detections')
        for chunk in tqdm(hdf5_in[dset].iter_chunks(), desc='chunks'):
            color_arr = hdf5_in[dset][chunk]
            keypoints = process_frames(color_arr)
            keypoints_dset[chunk[0], :, :] = keypoints[:, :, 0:2]
            confidence_dset[chunk[0], :] = keypoints[:, :, 2]

    print('Adding Stereo Depth')
    add_stereo_depth(
        hdf5_in, hdf5_out, cam_root,
        transforms[source][cam] if (source in transforms and
                                    cam in transforms[source]) else None)
    print('Done Adding Stereo Depth')

    print('done processing')
    hdf5_in.close()
    hdf5_out.close()
    print('done closing')


if __name__ == '__main__':
    PARSER = argparse.ArgumentParser()

    PARSER.add_argument("-v", "--video", type=str, required=True,
                        help="the hdf5 file with video data")
    PARSER.add_argument("-n", "--no_video", type=str, required=True,
                        help="the hdf5 file without video data")
    PARSER.add_argument("-t", "--transforms", type=str, required=True,
                        help="the transforms file")
    PARSER.add_argument("-s", "--source", choices=['podium', 'robot', 'mixed'],
                        required=True, help="Where this data was recorded")
    PARSER.add_argument("-c", "--cam", choices=['lower', 'upper'], default='lower',
                        help="Which camera to process")
    PARSER.add_argument("--rerun", dest='rerun', action='store_true',
                        help="re-run pose detection even if keypoints " +
                        "already exist in output")
    PARSER.add_argument("--no-rerun", dest='rerun', action='store_false',
                        help="Do not re-run pose detection if keypoints " +
                        "already exist in output")
    ARGS = PARSER.parse_args()
    convert(ARGS.video, ARGS.no_video, ARGS.transforms,
            ARGS.source, ARGS.cam, ARGS.rerun)
