#!/usr/bin/env python3

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
from utils import draw_text
from utils import colorScale
import kinematics.scripts.extract_profiles as k
import copy

# Inspiration from: https://medium.com/@pnpsegonne/animating-a-3d-scatterplot-with-matplotlib-ca4b676d4b55

def animate(iteration, frames, hdf5_tracking, length, plots):
    all_plots = []
    for topic, plot in plots.items():
        if(topic == 'img_plot'):
            plot.set_array(frames[iteration])
        else:
            y_data = np.zeros(length)
            y_data[:length] = hdf5_tracking[topic][iteration:iteration + length] 
            plot.set_ydata(y_data)

        all_plots.append(plot)

    return all_plots


def overlay_angular_motion(file_stub, cam, save = False, show = False):
    hdf5_video = h5py.File(file_stub+'.hdf5', 'r')
    hdf5_tracking = h5py.File(file_stub+'-novid.hdf5', 'r')
    color_dset = 'vid/color/data/{}/data'.format(cam)
    frames = hdf5_video[color_dset]
    print(frames.shape)

    length = 10
    plots = {}
    topics = ['features/ls_z_rot', 'features/ls_x_rot', 'features/ls_y_rot' ,
                'features/rs_z_rot' ,'features/rs_x_rot' , 'features/rs_y_rot']

    fig, axs  = plt.subplots(nrows = 3, ncols = 3) 
    
    img_plot = axs[0][1].imshow(frames[0])
    plots['img_plot'] = img_plot

    
    idx = 0
    for j in range(1,3):
        for ax in axs[j]:
            y_data = np.zeros(length)
            y_data[:length] = hdf5_tracking[topics[idx]][:length]
            ax.set_xlim([0, 20])
            ax.set_ylim([-np.pi, np.pi])
            ax.set_ylabel(topics[idx].split('/')[-1])
            ax.set_xlabel('Time')
            plots[topics[idx]] = ax.plot(y_data)[0]
            idx = idx+1

    ani = animation.FuncAnimation(fig,
                                  animate,
                                  frames.shape[0]-length,
                                  fargs=(frames, hdf5_tracking, length, plots),
                                  interval=50)

    if save:
        writervideo = animation.FFMpegWriter(fps=60)
        ani.save(file_stub + 'angular_motion.avi', writer=writervideo)

    if show:
        plt.show()

    hdf5_video.close()
    hdf5_tracking.close()
