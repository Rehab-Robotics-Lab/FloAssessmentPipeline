#!/usr/bin/env python3

import pathlib
import h5py
from tqdm import trange
import cv2
import math
import sys
import numpy as np
from common import color
from common import img_overlays
from pose_body.scripts.openpose_joints import openpose_joints


def overlay_2dSkeleton(directory, cam):
    directory = pathlib.Path(directory)
    hdf5_video = h5py.File(directory/'full_data-vid.hdf5')
    hdf5_tracking = h5py.File(directory/'full_data-novid.hdf5')

    cam_root = f'vid/{cam}'
    color_dset_name = f'{cam_root}/color/data'
    video_writer = cv2.VideoWriter(
        str(directory/f'viz-{cam}-2dSkeleton.avi'),
        cv2.VideoWriter_fourcc(*'MJPG'), 30, (1920, 1080))

    for idx in trange(hdf5_video[color_dset_name].shape[0]):
        img = hdf5_video[color_dset_name][idx]
        keypoints = hdf5_tracking[f'{cam_root}/openpose/keypoints'][idx]
        confidence = hdf5_tracking[f'{cam_root}/openpose/confidence'][idx]
        time = hdf5_tracking[f'{cam_root}/color/time'][idx]

        img_overlays.draw_text(img, 'frame: {}'.format(idx), pos=(100, 3))
        img_overlays.draw_text(img, 'time: {:.2f}'.format(time), pos=(500, 3))
        img_overlays.draw_text(
            img, 'view: {} realsense'.format(cam), pos=(900, 3))

        # Joints listed here: https://github.com/CMU-Perceptual-Computing-Lab/openpose/
        # blob/master/doc/02_output.md#keypoints-in-cpython

        joints = [openpose_joints().index(key) for key in [
            "Nose", "LEye", "REye", "LEar", "REar", "LShoulder", "RShoulder",
            "LElbow", "RElbow", "LWrist", "RWrist", "LHip", "RHip", "UpperNeck",
            "HeadTop"
        ]
        ]

        for joint in joints:
            x_pos = int(keypoints[joint][0])
            y_pos = int(keypoints[joint][1])
            cv2.circle(img, (x_pos, y_pos), 10,
                       color.color_scale(confidence[joint], 0, 1), 8)

        pairs = [
            (openpose_joints().index('UpperNeck'),
                openpose_joints().index('LShoulder')),
            (openpose_joints().index('UpperNeck'),
                openpose_joints().index('RShoulder')),
            (openpose_joints().index('LShoulder'),
                openpose_joints().index('LElbow')),
            (openpose_joints().index('RShoulder'),
                openpose_joints().index('RElbow')),
            (openpose_joints().index('LElbow'),
                openpose_joints().index('LWrist')),
            (openpose_joints().index('RElbow'),
                openpose_joints().index('RWrist')),
            (openpose_joints().index('UpperNeck'),
                openpose_joints().index('LHip')),
            (openpose_joints().index('UpperNeck'),
                openpose_joints().index('RHip'))
        ]
        for joint in pairs:
            x_0 = int(keypoints[joint[0]][0])
            y_0 = int(keypoints[joint[0]][1])

            x_1 = int(keypoints[joint[1]][0])
            y_1 = int(keypoints[joint[1]][1])

            if confidence[joint[0]] > 0.5 and confidence[joint[1]] > 0.5:
                cv2.line(img, (x_0, y_0), (x_1, y_1), color.color_scale(
                    confidence[joint[0]] + confidence[joint[1]], 0, 1))

        video_writer.write(img)

    video_writer.release()
    hdf5_video.close()
    hdf5_tracking.close()
