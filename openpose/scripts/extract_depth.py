'''
Module to extract depth given hdf5_in and hdf5_out files
'''
import numpy as np
from scipy.signal import convolve2d
from tqdm import trange

def extract_depth(depth_img, keypoints, params, window_size = 3): #pylint: disable= too-many-locals
    '''
    Function to extract depth in the aligned camera frame given intrinsic and extrinsics
    '''
    inv_kc, k_d, r_cd, t_cd = params
    keypoints_with_depth = np.ones((keypoints.shape[0], keypoints.shape[1] + 1))
    keypoints_with_depth[:,:2] = keypoints  # Keypoints with depth appended

    #shift = (Kd @ np.asarray([[0.015],[0],[0]]))[0]

    keypoints_in_depth = (k_d @ (r_cd.T @ ((inv_kc @ keypoints_with_depth.T) - t_cd))).T
    keypoints_with_depth = (inv_kc @ keypoints_with_depth.T).T
    kernel = np.ones((window_size, window_size)) / window_size ** 2

    depth_avg = convolve2d(
        depth_img, kernel, mode='same')


    for i in range(keypoints_with_depth.shape[0]):

        #x = int(keypoints_in_depth[i, 0] - shift)
        p_x = int(keypoints_in_depth[i, 0])
        p_y = int(keypoints_in_depth[i, 1])

        if 0<=p_y<depth_avg.shape[0] and 0<=p_x<depth_avg.shape[1]:
            p_z = depth_avg[p_y, p_x]
        else:
            p_z = 0

        keypoints_with_depth[i] = keypoints_with_depth[i] * (p_z/1000)
    return keypoints_with_depth

def add_stereo_depth(hdf5_video, hdf5_tracking): # pylint: disable= too-many-locals
    '''
    Function to create datasets in hdf5_tracking(hdf5_out) and add 3d keypoints
    '''
    for cam in ['lower', 'upper']:
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

        k_c = hdf5_video[color_dset].attrs['K'].reshape(3, 3)
        r_cd = hdf5_video[color_dset].attrs['depth_to_color-rotation'].reshape(3, 3)
        t_cd = hdf5_video[color_dset].attrs['depth_to_color-translation'].reshape(3, 1)

        inv_kc = np.linalg.inv(k_c)

        k_d = hdf5_video[depth_dset].attrs['K'].reshape(3, 3)

        for idx in trange(hdf5_video[color_dset].shape[0]):
            matched_index = hdf5_video[depth_match_dset][idx]
            depth_img = hdf5_video[depth_dset][matched_index]
            keypoints = hdf5_tracking[color_dset + '-keypoints'][idx]
            color_img = hdf5_video[color_dset][idx]
            keypoints3d_dset[idx, :, :] = extract_depth(depth_img,
                                                        keypoints,
                                                        (inv_kc,
                                                        k_d,
                                                        color_img,
                                                        r_cd,
                                                        t_cd))
