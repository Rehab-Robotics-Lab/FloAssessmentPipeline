#!/usr/bin/env python3
"""Module to extract pose using openpose from hdf5 file"""

import argparse
import json
import numpy as np
import h5py
from extract_depth import add_stereo_depth
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


def convert(video_pth, no_video_pth, transforms_pth, source, cam, rerun, algorithm):
    # pylint: disable= too-many-statements
    # pylint: disable= too-many-arguments
    # pylint: disable= too-many-locals
    # pylint: disable= too-many-branches
    """Extract poses from video in hdf5 file and build new hdf
    file with poses, confidences, and other non-video data.

    Args:
        video_pth: the path of the hdf5 source file with images/video to work with
                   (will not write here, only read)
        no_video_pth: the path of the hdf5 file without image/video data
                      (will write poses out to here)
        transforms_pth: Path to transforms file as generated by `get_transforms`
        source: Where the file came from. Options are: 'mixed', 'podium',
                and 'robot'
        cam: The camera to process. Options are `lower` and `upper`
        rerun: Whether to rerun the pose detection if a keypoints dataset
               already exists in the hdf5_out file
        algorithm: Which pose detection algorithm to run. Options are:
                   `openpose` (openpose body and hands),
                   `mp-hands` (mediapipe hands), `detectron2`
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
    color_dset = f'{cam_root}/color/data'
    pose_dset_root = f'{cam_root}/pose/{algorithm}'

    if algorithm == "mp-hands":
        num_keypoints = 20
    elif algorithm == "openpose":
        num_keypoints = 25+2*21  # 25 for body_25b and 21 for each hand
    else:
        raise ValueError("invalid algorithm passed")

    preexisting_keypoints = False
    kp_dset_name = f'{pose_dset_root}/keypoints'
    if kp_dset_name not in hdf5_out:
        keypoints_dset = hdf5_out.create_dataset(
            kp_dset_name, (hdf5_in[color_dset].len(), num_keypoints, 2), dtype=np.float32)
    else:
        keypoints_dset = hdf5_out[kp_dset_name]
        preexisting_keypoints = True
        print('keypoints already exist')

    preexisting_confidence = False
    conf_dset_name = f'{pose_dset_root}/confidence'
    if conf_dset_name not in hdf5_out:
        confidence_dset = hdf5_out.create_dataset(
            conf_dset_name, (hdf5_in[color_dset].len(), num_keypoints), dtype=np.float32)
    else:
        confidence_dset = hdf5_out[conf_dset_name]
        preexisting_confidence = True
        print('confidences already exist')

    if (not(preexisting_keypoints and preexisting_confidence)) or rerun:
        print('running pose detections')
        if algorithm == "openpose":
            # for openpose, keypoints will be:
            # body_25b, left hand, right hand
            # We only want to import if we are doing openpose. We could alternatively
            # put a wrapper around this whole thing. But since we aren't doing that,
            # we don't want to need imports for openpose for a different algorithm
            # pylint: disable=import-outside-toplevel
            from openpose_wrapper import process_frames
            for chunk in tqdm(hdf5_in[color_dset].iter_chunks(), desc='chunks'):
                color_arr = hdf5_in[color_dset][chunk]
                keypoints, params = process_frames(color_arr)
                keypoints_dset[chunk[0], :, :] = keypoints[:, :, 0:2]
                confidence_dset[chunk[0], :] = keypoints[:, :, 2]
                for key, val in params.items():
                    keypoints_dset.attrs[key] = val
                    confidence_dset.attrs[key] = val

    print('Adding Stereo Depth')
    add_stereo_depth(
        hdf5_in, hdf5_out, cam_root, pose_dset_root,
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
    PARSER.add_argument("-a", "--algorithm",
                        choices=['openpose', 'mp-hands', 'detectron2'],
                        required=True,
                        help="Algorithm to run for pose estimation")
    ARGS = PARSER.parse_args()
    convert(ARGS.video, ARGS.no_video, ARGS.transforms,
            ARGS.source, ARGS.cam, ARGS.rerun, ARGS.algorithm)
