#!/usr/bin/env python3
"""Module to extract kinematic measures from previously processed hdf5 files"""

import sys
import numpy as np
import extract_profiles as ep
import h5py

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
        hdf5_tracking = h5py.File(pth, 'r')
    except:  # pylint: disable=bare-except
        print('HDF5 database could not be read')
        raise

    print('opened hdf file')

    topic_3dkeypoints = 'vid/color/data/lower/data-3dkeypoints-stereo'
    keypoints  = hdf5_tracking[topic_3dkeypoints]
    print(keypoints.shape)

    right_wrist_kp = extract_right_wrist_points(keypoints)
    left_wrist_kp = extract_left_wrist_points(keypoints)

    rw_vel = ep.velocity_profile(right_wrist_kp, timestamps)
    lw_vel = ep.velocity_profile(left_wrist_kp, timestamps)

    rw_range_mov = ep.range_of_movement(right_wrist_kp)
    lw_range_mov = ep.range_of_movement(left_wrist_kp)

    rw_jerk = ep.jerk(rw_vel, timestamps)
    lw_jerk = ep.jerk(lw_vel, timestamps)

    rw_time_to_max = ep.time_to_max_velocity(rw_vel, timestamps)
    lw_time_to_max = ep.time_to_max_velocity(lw_vel, timestamps)

    rw_range_motion = ep.range_of_motion(rw_vel, timestamps)
    lw_range_motion = ep.range_of_motion(lw_vel, timestamps)

    #Make Feature Vector here (8 Features)

    print('done processing')
    hdf5_tracking.close()
    print('done closing')


if __name__ == '__main__':
    extract_kinematics(sys.argv[1])