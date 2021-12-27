'''
Module to extract depth given hdf5_in and hdf5_out files
'''
import numpy as np
from scipy.signal import convolve2d
from tqdm import trange


def extract_depth(depth_img, keypoints, params, window_size=3):  # pylint: disable= too-many-locals
    '''
    Function to extract depth in the aligned camera frame given intrinsic and extrinsics
    '''
    inv_kc, k_d, r_cd, t_cd = params
    keypoints_with_depth = np.ones(
        (keypoints.shape[0], keypoints.shape[1] + 1))
    keypoints_with_depth[:, :2] = keypoints  # Keypoints with depth appended

    #shift = (Kd @ np.asarray([[0.015],[0],[0]]))[0]

    keypoints_in_depth = (
        k_d @ (r_cd.T @ ((inv_kc @ keypoints_with_depth.T) - t_cd))).T
    keypoints_with_depth = (inv_kc @ keypoints_with_depth.T).T
    kernel = np.ones((window_size, window_size)) / window_size ** 2

    depth_avg = convolve2d(
        depth_img, kernel, mode='same')

    for i in range(keypoints_with_depth.shape[0]):

        #x = int(keypoints_in_depth[i, 0] - shift)
        p_x = int(keypoints_in_depth[i, 0])
        p_y = int(keypoints_in_depth[i, 1])

        if 0 <= p_y < depth_avg.shape[0] and 0 <= p_x < depth_avg.shape[1]:
            p_z = depth_avg[p_y, p_x]
        else:
            p_z = 0

        keypoints_with_depth[i] = keypoints_with_depth[i] * (p_z/1000)
    return keypoints_with_depth


def add_stereo_depth(hdf5_in, hdf5_out, cam_root, transforms=None):  # pylint: disable= too-many-locals
    '''
    Function to create datasets in hdf5_tracking(hdf5_out) and add 3d keypoints
    '''
    color_dset_name = f'{cam_root}/color/data'
    depth_match_dset_name = f'{cam_root}/color/matched_depth_index'
    time_dset_name = f'{cam_root}/color/time'
    depth_dset_name = f'{cam_root}/depth/data'
    keypoints2d_dset_name = f'{cam_root}/openpose/keypoints'
    keypoints3d_dset_name = f'{cam_root}/openpose/keypoints-3d'
    keypoints3d_dset = None

    if keypoints3d_dset_name not in hdf5_out:
        keypoints3d_dset = hdf5_out.create_dataset(
            keypoints3d_dset_name, (hdf5_in[color_dset_name].len(), 25, 3), dtype=np.float32)
    else:
        keypoints3d_dset = hdf5_out[keypoints3d_dset_name]
        print('You might be running the Stereo depth extraction twice')

    k_c = hdf5_in[color_dset_name].attrs['K'].reshape(3, 3)

    if ('depth_to_color-rotation' in hdf5_in[color_dset_name].attrs and
            'depth_to_color-translation' in hdf5_in[color_dset_name].attrs):
        r_cd_raw = hdf5_in[color_dset_name].attrs['depth_to_color-rotation']
        t_cd_raw = hdf5_in[color_dset_name].attrs['depth_to_color-translation']
    else:
        # transforms is already the
        time_target = hdf5_in[time_dset_name][0] + \
            (hdf5_in[time_dset_name][-1] - hdf5_in[time_dset_name][0])/2
        transform_idx = np.argmin(
            np.abs(np.asarray([float(v) for v in transforms.keys()])-time_target))
        best_transform = transforms[[*transforms][transform_idx]]
        r_cd_raw = best_transform['rotation']
        t_cd_raw = best_transform['translation']

    r_cd = np.asarray(r_cd_raw).reshape(3, 3)
    t_cd = np.asarray(t_cd_raw).reshape(3, 1)

    inv_kc = np.linalg.inv(k_c)

    k_d = hdf5_in[depth_dset_name].attrs['K'].reshape(3, 3)

    for idx in trange(hdf5_in[color_dset_name].shape[0]):
        matched_index = hdf5_in[depth_match_dset_name][idx]
        depth_img = hdf5_in[depth_dset_name][matched_index]
        keypoints = hdf5_out[keypoints2d_dset_name][idx]
        keypoints3d_dset[idx, :, :] = \
            extract_depth(depth_img, keypoints,
                          (inv_kc, k_d, r_cd, t_cd))
