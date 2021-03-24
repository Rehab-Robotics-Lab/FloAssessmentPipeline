#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
import matplotlib.animation as animation
import h5py
from tqdm import tqdm, trange
import cv2
import math
import sys
from utils import draw_text
from utils import colorScale

def skeleton3d(file_stub, cam):
    hdf5_video = h5py.File(file_stub+'.hdf5')
    hdf5_tracking = h5py.File(file_stub+'-novid.hdf5')

    dset = 'vid/color/data/{}/data'.format(cam)
    depth_match_dset = 'vid/color/data/{}/depth_match'.format(cam)
    depth_dset = "vid/depth/data/{}/data".format(cam)

    video_writer = cv2.VideoWriter(
        file_stub+'-'+cam+'-2dskel.avi', cv2.VideoWriter_fourcc(*'MJPG'), 30, (1920, 1080))

    for idx in trange(hdf5_video[dset].shape[0]):
        img = hdf5_video[dset][idx]
        keypoints = hdf5_tracking[dset+'-keypoints'][idx]
        confidence = hdf5_tracking[dset+'-confidence'][idx]
        sec = hdf5_tracking['vid/color/data/{}/secs'.format(cam)][idx]
        nsec = hdf5_tracking['vid/color/data/{}/nsecs'.format(cam)][idx]
        time = sec+nsec*1e-9


    hdf5_video.close()
    hdf5_tracking.close()

def animate_scatters(iteration, data, scatters):
    """
    Update the data held by the scatter plot and therefore animates it.
    Args:
        iteration (int): Current iteration of the animation
        data (list): List of the data positions at each iteration.
        scatters (list): List of all the scatters (One per element)
    Returns:
        list: List of scatters (One per element) with new coordinates
    """
    for i in range(data[0].shape[0]):
        scatters[i]._offsets3d = (data[iteration][i,0:1], data[iteration][i,1:2], data[iteration][i,2:])
    return scatters

def main(data, save=False):

    fig = plt.figure()
    ax = p3.Axes3D(fig)
    scatters = [ ax.scatter(data[0][i,0:1], data[0][i,1:2], data[0][i,2:]) for i in range(data[0].shape[0]) ]
    iterations = len(data)
    ax.set_xlim3d([-50, 50])
    ax.set_xlabel('X')

    ax.set_ylim3d([-50, 50])
    ax.set_ylabel('Y')

    ax.set_zlim3d([-50, 50])
    ax.set_zlabel('Z')

    ax.set_title('3D Skeleton')

    ax.view_init(25, 10)

    ani = animation.FuncAnimation(fig, animate, iterations, fargs=(data, scatters),
                                       interval=50, blit=False, repeat=True)

    if save:
        ani.save('3d-scatted-animated.gif', writer='imagemagick')
    plt.show()


main(data, save=False)