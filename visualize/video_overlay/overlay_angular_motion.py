#!/usr/bin/env python3

import pathlib
from typing import Counter
import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
import matplotlib.animation as animation
import h5py
from numpy.core.fromnumeric import shape
import cv2
import math
import sys
from common import img_overlays
import kinematics.scripts.extract_profiles as k
from tqdm import trange
import copy


def overlay_angular_motion(directory, cam):  # pylint: disable=too-many-locals
    """overlay data from hdf5 file onto images from hdf file.
    Requires both the no video and video hdf5 files.
    Args:
        directory: The common file stub for the two hdf5 files
        cam: The camera to use (upper or lower)
    """
    directory = pathlib.Path(directory)
    hdf5_video = h5py.File(directory/'full_data-vid.hdf5')
    hdf5_tracking = h5py.File(directory/'full_data-novid.hdf5')

    cam_root = f'vid/{cam}'
    color_dset_name = f'{cam_root}/color/data'
    video_writer = cv2.VideoWriter(
        str(directory/f'viz-{cam}-angular_motion.avi'),
        cv2.VideoWriter_fourcc(*'MJPG'), 30, (1920, 1080))
    t_vals = np.zeros(500, dtype=np.float64)

    ls_rot_topics = ['features/ls_z_rot',
                     'features/ls_x_rot', 'features/ls_y_rot']
    rs_rot_topics = ['features/rs_z_rot',
                     'features/rs_x_rot', 'features/rs_y_rot']

    ls_vel_topics = ['features/ls_z_vel',
                     'features/ls_x_vel', 'features/ls_y_vel']
    rs_vel_topics = ['features/rs_z_vel',
                     'features/rs_x_vel', 'features/rs_y_vel']

    ls_acc_topics = ['features/ls_z_acc',
                     'features/ls_x_acc', 'features/ls_y_acc']
    rs_acc_topics = ['features/rs_z_acc',
                     'features/rs_x_acc', 'features/rs_y_acc']

    scales = {}
    scales['rot'] = 20
    scales['vel'] = 1e9 * 10
    scales['acc'] = 1e17 * 10
    scales['angle'] = 30

    # Relative movement with previous quaternion
    full_arm_rot_ls_topic = ['features/ls_angle']
    full_arm_rot_rs_topic = ['features/rs_angle']

    all_ls_topics = [ls_rot_topics, ls_vel_topics,
                     ls_acc_topics, full_arm_rot_ls_topic]
    all_ls_vals = []
    for topics in all_ls_topics:
        vals = []
        for topic in topics:
            vals.append(np.ones(500, dtype=np.float64))
        all_ls_vals.append(vals)

    all_rs_topics = [rs_rot_topics, rs_vel_topics,
                     rs_acc_topics, full_arm_rot_rs_topic]
    all_rs_vals = []
    for topics in all_rs_topics:
        vals = []
        for topic in topics:
            vals.append(np.ones(500, dtype=np.float64))
        all_rs_vals.append(vals)

    for idx in trange(hdf5_video[color_dset_name].shape[0]-2):
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
        for joint in (2, 3, 4, 5, 6, 7):
            x = int(keypoints[joint][0])  # pylint: disable=invalid-name
            y = int(keypoints[joint][1])  # pylint: disable=invalid-name
            cv2.circle(img, (x, y), 20,
                       img_overlays.color_scale(confidence[4], 0, 1), 8)

        # TODO: get this all in a clean loop
        # TODO: plot on top of each other
        t_vals = np.roll(t_vals, 1)
        t_vals[0] = time

        adj_time = t_vals - time
        in_range = adj_time > -15
        scaled_time = adj_time * 30 + 450

        y_offset = 0
        for i, topics in enumerate(all_ls_topics):
            y_offset = y_offset + 200
            x_offset = 1500
            for j, topic in enumerate(topics):
                scale = scales[topic.split('_')[-1]]
                all_ls_vals[i][j] = np.roll(all_ls_vals[i][j], 1)
                if(math.isnan(hdf5_tracking[topic][idx])):
                    img_overlays.draw_text(
                        img, 'Outlier ', pos=(x_offset, 100))
                    print('here')
                else:
                    all_ls_vals[i][j][0] = hdf5_tracking[topic][idx]

                color = np.zeros(3)
                color[j] = 255

                img_overlays.draw_text(img, topic.split('/')[-1], pos=(x_offset, y_offset - 100 + 30*j),
                                       font_scale=1,
                                       font_thickness=1,
                                       text_color=color,
                                       text_color_bg=None)

                cv2.polylines(img,
                              [np.int32(
                                  np.transpose(
                                      (x_offset + scaled_time[in_range],
                                       (y_offset + scale * all_ls_vals[i][j][in_range]))))],
                              isClosed=False, color=color, thickness=4)

        y_offset = 0
        for i, topics in enumerate(all_rs_topics):
            y_offset = y_offset + 200
            x_offset = 0
            for j, topic in enumerate(topics):
                scale = scales[topic.split('_')[-1]]
                all_rs_vals[i][j] = np.roll(all_rs_vals[i][j], 1)
                if(math.isnan(hdf5_tracking[topic][idx])):
                    img_overlays.draw_text(
                        img, 'Outlier ', pos=(x_offset, 100))
                else:
                    all_rs_vals[i][j][0] = hdf5_tracking[topic][idx]

                color = np.zeros(3)
                color[j] = 255
                img_overlays.draw_text(img, topic.split('/')[-1], pos=(x_offset, y_offset - 100 + 30*j),
                                       font_scale=1,
                                       font_thickness=1,
                                       text_color=color,
                                       text_color_bg=None)

                cv2.polylines(img,
                              [np.int32(
                                  np.transpose(
                                      (x_offset + scaled_time[in_range],
                                       (y_offset + scale * all_rs_vals[i][j][in_range]))))],
                              isClosed=False, color=color, thickness=4)

        video_writer.write(img)
    video_writer.release()
    hdf5_video.close()
    hdf5_tracking.close()


# Expect one argument of form: <file stub> which will be used to access
# <file stub>.hdf5 and <filestub>-novid.hdf5 and create <file stub>-wrists.avi
if __name__ == '__main__':
    overlay_angular_motion(sys.argv[1], sys.argv[2])
