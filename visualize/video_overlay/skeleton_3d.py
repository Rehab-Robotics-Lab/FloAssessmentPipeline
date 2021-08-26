#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import proj3d
import h5py
from tqdm import tqdm, trange
import cv2
import math
import sys
from utils import draw_text
from utils import colorScale
import kinematics.extract_profiles as k
import copy

# Inspiration from: https://medium.com/@pnpsegonne/animating-a-3d-scatterplot-with-matplotlib-ca4b676d4b55


def animate(iteration, data, scatters, lines, texts, ax, joint_pairs, keypoints):
    for i in keypoints:
        scatters[i]._offsets3d = (data[iteration, i, 0:1], data[iteration, i, 1:2], data[iteration, i, 2:])

    for i, line in enumerate(lines):
        line.set_data([data[iteration, joint_pairs[i][0], 0], data[iteration, joint_pairs[i][1], 0]],
                      [data[iteration, joint_pairs[i][0], 1], data[iteration, joint_pairs[i][1], 1]])
        line.set_3d_properties([data[iteration, joint_pairs[i][0], 2], data[iteration, joint_pairs[i][1], 2]])

    for i, text in enumerate(texts):
        x, y, _ = proj3d.proj_transform(data[iteration, i, 0], data[iteration, i, 1], data[iteration, i, 2], ax.get_proj())
        text.set_position((x, y))

    return scatters, lines, texts


def skeleton_3d(file_stub, cam, save = False, show = False):
    hdf5_video = h5py.File(file_stub+'.hdf5', 'r')
    hdf5_tracking = h5py.File(file_stub+'-novid.hdf5', 'r')
    color_dset = 'vid/color/data/{}/data'.format(cam)

    points3d = hdf5_tracking[color_dset + '-3dkeypoints-stereo']

    keypoints = [4, 3, 2, 1, 5, 6, 7, 8, 0]
    joint_pairs = [(0, 1), (4, 3), (3, 2), (2, 1), (1, 5), (5, 6), (6, 7), (1, 8)]
    annotations = ['nose', 'neck', 'lshoulder', 'lelbow', 'lwrist', 'rshoulder', 'relbow', 'rwrists', 'waist']

    fig = plt.figure()
    ax = p3.Axes3D(fig)

    scatters = [ ax.scatter(points3d[0, i, 0:1], points3d[0, i, 1:2], points3d[0, i, 2:]) for i in keypoints]

    lines = [ ax.plot([points3d[0, joint[0], 0], points3d[0, joint[1], 0]],
                            [points3d[0, joint[0], 1], points3d[0, joint[1], 1]],
                            [points3d[0, joint[0], 2], points3d[0, joint[1], 2]],
                            'black')[0] for joint in joint_pairs]

    texts = [ax.text2D(points3d[0][i, 0], points3d[0][i, 1],
             '%s' % (annotations[i]),
             size=5,
             zorder=1, color='k') for i in range(len(annotations))]


    frames = {}

    r_fixed_frame = k.right_shoulder_fixed_frame(points3d[0])
    rs_f = [ax.quiver(points3d[0, , 2],) for i in range(3)]

    ax.set_xlim3d([-1, 1])
    ax.set_xlabel('X')

    ax.set_ylim3d([-1, 1])
    ax.set_ylabel('Y')

    ax.set_zlim3d([0, 5])
    ax.set_zlabel('Z')

    ax.set_title('3D Skeleton')
    ax.view_init(-90, -86)

    ani = animation.FuncAnimation(fig,
                                  animate,
                                  len(points3d),
                                  fargs=(points3d, scatters, lines, texts, ax, joint_pairs, keypoints),
                                  interval=50)

    if save:
        writervideo = animation.FFMpegWriter(fps=60)
        ani.save(file_stub + '3d-skeleton.avi', writer=writervideo)

    if show:
        plt.show()

    hdf5_video.close()
    hdf5_tracking.close()
