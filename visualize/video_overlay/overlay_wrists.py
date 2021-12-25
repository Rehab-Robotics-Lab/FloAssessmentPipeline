#!/usr/bin/env python3

import pathlib
import h5py
from tqdm import tqdm, trange
import cv2
import math
import sys
import numpy as np
from common import img_overlays


def overlay_wrists(directory, cam):
    directory = pathlib.Path(directory)
    hdf5_video = h5py.File(directory/'full_data-vid.hdf5')
    hdf5_tracking = h5py.File(directory/'full_data-novid.hdf5')

    dset = 'vid/color/data/{}/data'.format(cam)
    video_writer = cv2.VideoWriter(
        directory/f'viz-{cam}-wrists.avi', cv2.VideoWriter_fourcc(*'MJPG'), 30, (1920, 1080))
    for idx in trange(hdf5_video[dset].shape[0]):
        img = hdf5_video[dset][idx]
        keypoints = hdf5_tracking[dset+'-keypoints'][idx]
        confidence = hdf5_tracking[dset+'-confidence'][idx]
        sec = hdf5_tracking['vid/color/data/{}/secs'.format(cam)][idx]
        nsec = hdf5_tracking['vid/color/data/{}/nsecs'.format(cam)][idx]
        time = sec+nsec*1e-9

        img_overlays.draw_text(img, 'frame: {}'.format(idx), pos=(100, 3))
        img_overlays.draw_text(img, 'time: {:.2f}'.format(time), pos=(500, 3))
        img_overlays.draw_text(
            img, 'view: {} realsense'.format(cam), pos=(900, 3))

        # Joints listed here: https://github.com/CMU-Perceptual-Computing-Lab/openpose/
        # blob/master/doc/02_output.md#keypoints-in-cpython
        for joint in (4, 7):
            x = int(keypoints[joint][0])
            y = int(keypoints[joint][1])
            cv2.circle(img, (x, y), 20,
                       img_overlays.colorScale(confidence[4], 0, 1), 8)

        video_writer.write(img)
    video_writer.release()
    hdf5_video.close()
    hdf5_tracking.close()
