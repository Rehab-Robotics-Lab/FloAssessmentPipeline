#!/usr/bin/env python3
"""Module to extract kinematic measures from previously processed hdf5 files"""

import sys
import numpy as np
from numpy.linalg import norm
import extract_profiles as ep
from scipy.spatial.transform import Rotation as R
from scipy.signal import convolve
import matplotlib.pyplot as plt
import h5py

CHUNK_SIZE = 500

def create_dataset(hdf5_obj, topics, shape, chunks = (CHUNK_SIZE,), 
                    maxshape = (None,), data_type = np.float):
    for topic in topics:
        if topic not in hdf5_obj:
            hdf5_obj.create_dataset(topic, shape,
                                    maxshape = maxshape,
                                    dtype = data_type, chunks = chunks)

def extract_kinematics(pth):
    # pylint: disable= too-many-statements
    """
    Function to extract kinematics given a dataset
    Args:
        pth: the path of the hdf5 source file to work with
    """
    print('processing on {}'.format(pth))

    # open hdf5 file
    try:
        hdf5_tracking = h5py.File(pth, 'a')
    except:  # pylint: disable=bare-except
        print('HDF5 database could not be read')
        raise

    print('opened hdf file')

    #Parameters
    window  = 20 #Window for moving average
    threshold = 0.5

    rot_topics = ['features/ls_z_rot', 'features/ls_x_rot', 'features/ls_y_rot' ,
                'features/rs_z_rot' ,'features/rs_x_rot' , 'features/rs_y_rot']
    
    vel_topics = ['features/ls_z_vel', 'features/ls_x_vel', 'features/ls_y_vel' ,
                'features/rs_z_vel' ,'features/rs_x_vel' , 'features/rs_y_vel']

    acc_topics = ['features/ls_z_acc', 'features/ls_x_acc', 'features/ls_y_acc' ,
                'features/rs_z_acc' ,'features/rs_x_acc' , 'features/rs_y_acc']
    
    #Relative movement with previous quaternion
    full_arm_rot_topics = ['features/ls_angle', 'features/rs_angle'] 
    quaternion_topics = ['left_shoulder_quat', 'right_shoulder_quat']


    topic_3dkeypoints = 'vid/color/data/lower/data-3dkeypoints-stereo'
    
    timestamps = np.asarray(hdf5_tracking['vid/color/data/lower/secs']) * 1e-9 + \
                 np.asarray(hdf5_tracking['vid/color/data/lower/nsecs'])

    keypoints  = hdf5_tracking[topic_3dkeypoints]
        
    #Creating and population HDF5 Dataset
    create_dataset(hdf5_tracking, rot_topics, keypoints.shape)
    create_dataset(hdf5_tracking, vel_topics, (keypoints.shape[0]-1,))
    create_dataset(hdf5_tracking, acc_topics, (keypoints.shape[0]-2,))
    create_dataset(hdf5_tracking, full_arm_rot_topics, (keypoints.shape[0]-1,))
    create_dataset(hdf5_tracking, quaternion_topics, (4,keypoints.shape[0],), (4,CHUNK_SIZE,), (None, None, ))

    prev_r_right_shoulder, prev_r_left_shoulder = ep.shoulder_angular_motion(keypoints[0])
    hdf5_tracking[quaternion_topics[0]][:,0] = prev_r_left_shoulder.as_quat()
    hdf5_tracking[quaternion_topics[1]][:,0] = prev_r_right_shoulder.as_quat()

    diff = []
    #Outlier rejection
    for i in range(1, keypoints.shape[0]):
        r_right_shoulder, r_left_shoulder = ep.shoulder_angular_motion(keypoints[i])

        angle_right = min(np.arccos(np.dot(prev_r_right_shoulder.as_quat(), r_right_shoulder.as_quat()))/2,
                      np.arccos(np.dot(prev_r_right_shoulder.as_quat(), -1*r_right_shoulder.as_quat()))/2)

        angle_left = min(np.arccos(np.dot(prev_r_left_shoulder.as_quat(), r_left_shoulder.as_quat()))/2,
                    np.arccos(np.dot(prev_r_left_shoulder.as_quat(), -1*r_left_shoulder.as_quat()))/2)

        diff.append(angle_right)
        diff.append(angle_left)

        #Full arm Rotation
        hdf5_tracking[full_arm_rot_topics[0]][i-1] = angle_left
        hdf5_tracking[full_arm_rot_topics[1]][i-1] = angle_right

        
        hdf5_tracking[quaternion_topics[0]][:,i] = r_left_shoulder.as_quat()
        hdf5_tracking[quaternion_topics[1]][:,i] = r_right_shoulder.as_quat()

        euler_ls = r_left_shoulder.as_euler('zxy')
        euler_rs = r_right_shoulder.as_euler('zxy')
        
        if(angle_right > threshold):
            euler_rs[:] = np.nan
        else: 
            prev_r_right_shoulder = r_right_shoulder
        
        if(angle_left > threshold):
            euler_ls[:] = np.nan
        else:
            prev_r_left_shoulder = r_left_shoulder

        all_eulers = np.concatenate((euler_ls, euler_rs))
        for j, topic in enumerate(rot_topics):
            hdf5_tracking[topic][i] = all_eulers[j]

    fig = plt.figure()
    plt.scatter(np.arange(len(diff)), diff)
    plt.show()
   
    #Smoothing and differentials
    for i, rot_topic in enumerate(rot_topics):

        smoothed_signal = convolve(hdf5_tracking[rot_topic], np.ones(window)/window, mode= 'same')
        hdf5_tracking[rot_topic][:] = smoothed_signal
        hdf5_tracking[vel_topics[i]][:smoothed_signal.shape[0]-1] = ep.diff(smoothed_signal, timestamps)
        hdf5_tracking[acc_topics[i]][:smoothed_signal.shape[0]-2] = ep.diff(ep.diff(smoothed_signal, timestamps), timestamps[1:])

        fig, axs = plt.subplots(ncols = 3)
        axs[0].plot(hdf5_tracking[rot_topic])
        axs[1].plot(hdf5_tracking[vel_topics[i]])
        axs[2].plot(hdf5_tracking[acc_topics[i]])
        plt.show()

    print('done processing')
    hdf5_tracking.close()
    print('done closing')


if __name__ == '__main__':
    extract_kinematics(sys.argv[1])