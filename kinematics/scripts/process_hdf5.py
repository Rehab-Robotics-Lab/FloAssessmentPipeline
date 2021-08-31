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

    topic_3dkeypoints = 'vid/color/data/lower/data-3dkeypoints-stereo'
    timestamps = np.asarray(hdf5_tracking['vid/color/data/lower/secs']) * 1e-9 + \
                 np.asarray(hdf5_tracking['vid/color/data/lower/nsecs'])

    keypoints  = hdf5_tracking[topic_3dkeypoints]
    
    r_right_shoulder, r_left_shoulder = ep.shoulder_angular_motion(keypoints[0])
    all_right_shoulder = [r_right_shoulder]
    all_left_shoulder = [r_left_shoulder]
    threshold = 0.1

    #Creating and population HDF5 Dataset
    topics = ['features/ls_z_rot', 'features/ls_x_rot', 'features/ls_y_rot' ,
                'features/rs_z_rot' ,'features/rs_x_rot' , 'features/rs_y_rot']

    for topic in topics:
        if topic not in hdf5_tracking:
            hdf5_tracking.create_dataset(topic, (len(all_left_shoulder),),
                                    maxshape=(None,),
                                    dtype=np.float, chunks=(CHUNK_SIZE,))

    diff = []
    #Outlier rejection
    for i in range(1, keypoints.shape[0]):
        r_right_shoulder, r_left_shoulder = ep.shoulder_angular_motion(keypoints[i])

        angle_right = min(np.arccos(np.dot(all_right_shoulder[-1].as_quat(), r_right_shoulder.as_quat()))/2,
                      np.arccos(np.dot(all_right_shoulder[-1].as_quat(), -1*r_right_shoulder.as_quat()))/2)

        angle_left = min(np.arccos(np.dot(all_left_shoulder[-1].as_quat(), r_left_shoulder.as_quat()))/2,
                    np.arccos(np.dot(all_left_shoulder[-1].as_quat(), -1*r_left_shoulder.as_quat()))/2)

        diff.append(angle_right)
        diff.append(angle_left)

        if(angle_right < threshold):
            all_right_shoulder.append(r_right_shoulder)
        else:
            all_right_shoulder.append(all_right_shoulder[-1]) #Copy previous value

        if(angle_left < threshold):
            all_left_shoulder.append(r_left_shoulder)
        else:
            all_left_shoulder.append(all_left_shoulder[-1]) #Copy previous value
        
    
    fig = plt.figure()
    plt.scatter(np.arange(len(diff)), diff)
    plt.show()
   
    #Moving Average    
    window  = 5 #Window for moving average

    for i in range(len(all_left_shoulder)):
        euler_ls = all_left_shoulder[i].as_euler('zxy')
        euler_rs = all_right_shoulder[i].as_euler('zxy')
        all_eulers = np.concatenate((euler_ls, euler_rs))
        for j, topic in enumerate(topics):
            hdf5_tracking[topic][i] = all_eulers[j]

    for topic in topics:
        plt.plot(hdf5_tracking[topic])
        plt.show()
        smoothed = convolve(hdf5_tracking[topic], np.ones(window)/window, mode= 'same')
        hdf5_tracking[topic][:] = smoothed
        plt.plot(hdf5_tracking[topic])
        plt.show()

    print('done processing')
    hdf5_tracking.close()
    print('done closing')


if __name__ == '__main__':
    extract_kinematics(sys.argv[1])