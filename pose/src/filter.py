"""Module to filter pose data"""
import numpy as np
import scipy.signal

# IMAGE_WIDTH = 1920
# IMAGE_HEIGHT = 1080
# # Measurement function (we measure x and y positions)
# H = np.array([[1, 0, 0, 0],
#               [0, 1, 0, 0]])
# # Initial vals
# # initial assumed state, center of image
# X0 = np.array([IMAGE_WIDTH/2, IMAGE_HEIGHT/2, 0, 0])
# P0 = np.array([[(IMAGE_WIDTH/4)**2, 0, 0, 0],  # initially could be anywhere in frame
#                [0, (IMAGE_HEIGHT/4)**2, 0, 0],
#                [0, 0, 500, 0],  # no clue what our initial velocity uncertanty is
#                [0, 0, 0, 500]])
# # Transistion covariance
# Q = np.array([[0, 0, 0, 0],
#               [0, 0, 0, 0],
#               [0, 0, 1, 0],
#               [0, 0, 0, 1]])*80
# # Observation covariance
# R = np.eye(2)*(2**2)


def smooth_2d(hdf5_file, tracking_root, kernel_size=5):
    """Smooths the 2D pose estimation data using a median
    filter with a 5 element kernel

    Args:
        hdf5_file: File to work on
        tracking_root (str): The root of the pose tracking
    """
    keypoints_dset_name = f'{tracking_root}/keypoints/color'
    confidences_dset_name = f'{tracking_root}/confidence'
    filtered_dset_name = f'{tracking_root}/keypoints-median{kernel_size}/color'

    if filtered_dset_name not in hdf5_file:
        hdf5_file.create_dataset(
            filtered_dset_name, hdf5_file[keypoints_dset_name].shape, dtype=np.float32)

    for joint in range(hdf5_file[keypoints_dset_name].shape[1]):
        # confidence could be joint wise or for all joints
        if len(hdf5_file[confidences_dset_name].shape) == 1:
            confidence = hdf5_file[confidences_dset_name]
        else:
            confidence = hdf5_file[confidences_dset_name][:, joint]
        keypoints = np.ma.array(hdf5_file[keypoints_dset_name][:, joint, :])
        valid = np.any(keypoints != 0, axis=1) + confidence > .15
        keypoints[np.logical_not(valid)] = np.ma.masked
        keypoints_smooth = keypoints.copy()
        keypoints_smooth[~keypoints.mask[:, 0], :] = np.ma.array(scipy.signal.medfilt(
            keypoints[~keypoints.mask[:, 0], :], kernel_size=[kernel_size, 1]))
        keypoints_smooth.mask = keypoints.mask
        keypoints_smooth[keypoints_smooth.mask] = 0
        hdf5_file[filtered_dset_name][:, joint] = keypoints_smooth
