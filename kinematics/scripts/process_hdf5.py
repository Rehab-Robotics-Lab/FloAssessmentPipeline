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
    timestamps = np.asarray(hdf5_tracking['vid/color/data/lower/secs']) * 1e-9 + \
                 np.asarray(hdf5_tracking['vid/color/data/lower/nsecs'])

    keypoints  = hdf5_tracking[topic_3dkeypoints]
    print(keypoints.shape)

    right_wrist_kp = ep.extract_right_wrist_points(keypoints)
    left_wrist_kp = ep.extract_left_wrist_points(keypoints)

    print(right_wrist_kp.shape)
    print(left_wrist_kp.shape)

    rw_vel, timestamps_vel = ep.velocity_profile(right_wrist_kp, timestamps)
    lw_vel, timestamps_vel = ep.velocity_profile(left_wrist_kp, timestamps)

    print("rw_vel :", rw_vel.shape)
    print("lw_vel :", lw_vel.shape)

    rw_range_mov = ep.range_of_movement(right_wrist_kp)
    lw_range_mov = ep.range_of_movement(left_wrist_kp)

    print(rw_range_mov.shape)
    print(lw_range_mov.shape)

    rw_time_to_max = ep.time_to_max_velocity(rw_vel, timestamps_vel)
    lw_time_to_max = ep.time_to_max_velocity(lw_vel, timestamps_vel)

    print(rw_time_to_max.shape)
    print(lw_time_to_max.shape)

    rw_range_motion = ep.range_of_motion(rw_vel, timestamps_vel)
    lw_range_motion = ep.range_of_motion(lw_vel, timestamps_vel)

    print(rw_range_motion.shape)
    print(lw_range_motion.shape)

    rw_jerk = ep.jerk(rw_vel, timestamps_vel)
    lw_jerk = ep.jerk(lw_vel, timestamps_vel)

    print(rw_jerk.shape)
    print(lw_jerk.shape)

    #Make Feature Vector here (8 Features)

    print('done processing')
    hdf5_tracking.close()
    print('done closing')


if __name__ == '__main__':
    extract_kinematics(sys.argv[1])