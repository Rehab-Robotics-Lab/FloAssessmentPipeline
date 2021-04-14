import numpy as np
import h5py
from tqdm import tqdm, trange
import cv2
import math
import sys
import copy

def extract_depth(depth_img, keypoints, inv_Kc, Kd, color_img, window_size = 3):

    keypoints_with_depth = np.ones((9, 3))
    keypoints_with_depth[:,:2] = keypoints[:9, :]

    shift = (Kd @ np.asarray([[0.015],[0],[0]]))[0]

    keypoints_in_depth = (Kd @ (inv_Kc @ keypoints_with_depth.T)).T
    keypoints_with_depth = (inv_Kc @ keypoints_with_depth.T).T

    depth_img_ws = copy.deepcopy(depth_img)

    for joint in range(0, 9):
        x = int(keypoints[joint][0])
        y = int(keypoints[joint][1])
        cv2.circle(color_img, (x, y), 10,
                   colorScale(0.8, 0, 1), 8)

        xd = int(keypoints_in_depth[joint][0]) - shift
        yd = int(keypoints_in_depth[joint][1])

        cv2.circle(depth_img_ws, (xd+shift, yd), 10,
                   colorScale(0.8, 0, 1), 8)

        cv2.circle(depth_img, (xd, yd), 10,
                   colorScale(0.8, 0, 1), 8)

    for i in range(keypoints_with_depth.shape[0]):

        x = int(keypoints_in_depth[i, 0] - shift)
        y = int(keypoints_in_depth[i, 1])
        #print("x: %d y: %d" %(x,y))

        Z = np.mean(depth_img[ y - window_size : y + window_size,
                               x - window_size : x + window_size])

        keypoints_with_depth[i] = keypoints_with_depth[i] * (Z/1000)

    return keypoints_with_depth

def addStereoDepth(hdf5_video, hdf5_tracking):

    for cam in ['upper', 'lower']:
        color_dset = 'vid/color/data/{}/data'.format(cam)
        depth_match_dset = 'vid/color/data/{}/matched_depth_index'.format(cam)
        depth_dset = "vid/depth/data/{}/data".format(cam)
        stereo_depth_dset = color_dset + '-3dkeypoints-stereo'

        keypoints3d_dset = None

        if stereo_depth_dset not in hdf5_tracking:
            keypoints3d_dset = hdf5_tracking.create_dataset(
                stereo_depth_dset, (hdf5_video[color_dset].len(), 25, 3), dtype=np.float32)
        else:
            keypoints3d_dset = hdf5_tracking[stereo_depth_dset]
            print('You might be running the Stereo depth extraction twice')

        K_c = hdf5_video[color_dset].attrs['K'].reshape(3, 3)
        inv_Kc = np.linalg.inv(K_c)

        K_d = hdf5_video[depth_dset].attrs['K'].reshape(3, 3)

        for idx in trange(hdf5_video[color_dset].shape[0]):
            matched_index = hdf5_video[depth_match_dset][idx]
            depth_img = hdf5_video[depth_dset][matched_index]
            keypoints = hdf5_tracking[color_dset + '-keypoints'][idx]
            confidence = hdf5_tracking[color_dset + '-confidence'][idx]
            color_img = hdf5_video[color_dset][idx]
            keypoints3d_dset[idx, :, :] = extract_depth(depth_img, keypoints, inv_Kc, K_d, color_img)