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
from utils import color_scale
import kinematics.scripts.extract_profiles as k
from tqdm import trange
import copy

def overlay(file_stub, cam):  # pylint: disable=too-many-locals
    """overlay data from hdf5 file onto images from hdf file.
    Requires both the no video and video hdf5 files.
    Args:
        file_stub: The common file stub for the two hdf5 files
        cam: The camera to use (upper or lower)
    """
    hdf5_video = h5py.File(file_stub+'.hdf5')
    hdf5_tracking = h5py.File(file_stub+'-novid.hdf5')

    dset = 'vid/color/data/{}/data'.format(cam)
    video_writer = cv2.VideoWriter(
        file_stub+'-'+cam+'-shoulders.avi', cv2.VideoWriter_fourcc(*'MJPG'), 30, (1920, 1080))
    t_vals = np.zeros(500, dtype=np.float64)

    ls_rot_topics = ['features/ls_z_rot', 'features/ls_x_rot', 'features/ls_y_rot'] 
    rs_rot_topics = ['features/rs_z_rot' ,'features/rs_x_rot' , 'features/rs_y_rot']
    
    ls_vel_topics = ['features/ls_z_vel', 'features/ls_x_vel', 'features/ls_y_vel']
    rs_vel_topics = ['features/rs_z_vel' ,'features/rs_x_vel' , 'features/rs_y_vel']

    ls_acc_topics = ['features/ls_z_acc', 'features/ls_x_acc', 'features/ls_y_acc']
    rs_acc_topics = ['features/rs_z_acc' ,'features/rs_x_acc' , 'features/rs_y_acc']

    scales = {}
    scales['rot'] = 20
    scales['vel'] = 1e8 * 20
    scales['acc'] = 1e7 * 20
  
    #Relative movement with previous quaternion
    full_arm_rot_topics = ['features/ls_angle', 'features/rs_angle'] 
    quaternion_topics = ['left_shoulder_quat', 'right_shoulder_quat']

    all_ls_topics = [ls_rot_topics, ls_vel_topics, ls_acc_topics]
    all_ls_vals = []
    for topics in all_ls_topics:
        vals = []
        for topic in topics:
            vals.append(np.ones(500, dtype=np.float64))
        all_ls_vals.append(vals)

    all_rs_topics = [rs_rot_topics, rs_vel_topics, rs_acc_topics]
    all_rs_vals = []
    for topics in all_rs_topics:
        vals = []
        for topic in topics:
            vals.append(np.ones(500, dtype=np.float64))
        all_rs_vals.append(vals)

    for idx in trange(hdf5_video[dset].shape[0]-2):
        img = hdf5_video[dset][idx]
        keypoints = hdf5_tracking[dset+'-keypoints'][idx]
        confidence = hdf5_tracking[dset+'-confidence'][idx]
        sec = hdf5_tracking['vid/color/data/{}/secs'.format(cam)][idx]
        nsec = hdf5_tracking['vid/color/data/{}/nsecs'.format(cam)][idx]
        time = sec+nsec*1e-9

        draw_text(img, 'frame: {}'.format(idx), pos=(100, 3))
        draw_text(img, 'time: {:.2f}'.format(time), pos=(500, 3))
        draw_text(img, 'view: {} realsense'.format(cam), pos=(900, 3))

        # Joints listed here: https://github.com/CMU-Perceptual-Computing-Lab/openpose/
        # blob/master/doc/02_output.md#keypoints-in-cpython
        for joint in (2, 3, 4, 5, 6, 7):
            x = int(keypoints[joint][0])  # pylint: disable=invalid-name
            y = int(keypoints[joint][1])  # pylint: disable=invalid-name
            cv2.circle(img, (x, y), 20,
                       color_scale(confidence[4], 0, 1), 8)

        # TODO: get this all in a clean loop
        # TODO: plot on top of each other
        t_vals = np.roll(t_vals, 1)
        t_vals[0] = time

        adj_time = t_vals - time
        in_range = adj_time > -15
        scaled_time = adj_time * 30 + 450
        '''
        y_offset = 0
        for i, topics in enumerate(all_ls_topics):
            y_offset = y_offset + 300
            x_offset = 1500
            for j, topic in enumerate(topics):
                #plt.plot(hdf5_tracking[topic])
                #plt.show()

                scale = scales[topic.split('_')[-1]] 
                all_ls_vals[i][j] = np.roll(all_ls_vals[i][j],1) 
                if(hdf5_tracking[topic][idx] == np.nan):
                    draw_text(img, 'Outlier ', pos = (x_offset, 100)) 
                    print('here')
                else:
                    all_ls_vals[i][j][0] = hdf5_tracking[topic][idx]  
                
                color = np.zeros(3)
                color[j] = 255

                cv2.polylines(img,
                            [np.int32(
                                np.transpose(
                                    (x_offset + scaled_time[in_range],
                                    (y_offset + scale * all_ls_vals[i][j][in_range]))))],
                            isClosed=False, color=color, thickness=4)
        '''
        y_offset = 0
        for i, topics in enumerate(all_rs_topics):
            y_offset = y_offset + 300
            x_offset = 0
            for j, topic in enumerate(topics):
                scale = scales[topic.split('_')[-1]] 
                all_rs_vals[i][j] = np.roll(all_rs_vals[i][j],1) 
                if(hdf5_tracking[topic][idx] == np.nan):
                    draw_text(img, 'Outlier ', pos = (x_offset, 100)) 
                else:
                    all_rs_vals[i][j][0] = hdf5_tracking[topic][idx]  
                
                color = np.zeros(3)
                color[j] = 255

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
    overlay(sys.argv[1], sys.argv[2])
