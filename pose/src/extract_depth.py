'''
Module to extract depth given hdf5_in and hdf5_out files
'''
import multiprocessing
from functools import partial
import tqdm
import numpy as np
from common.realsense_params import MIN_VALID_DEPTH_METERS
from common.realsense_params import MAX_VALID_DEPTH_METERS
from common.tracking_params import DEPTH_KERNEL_SIZE
from common.tracking_params import MAX_TIME_DISPARITY

assert DEPTH_KERNEL_SIZE % 2 == 1
MIN_VALID_DEPTH_MM = MIN_VALID_DEPTH_METERS*1000
MAX_VALID_DEPTH_MM = MAX_VALID_DEPTH_METERS*1000


def generate_image_h_indices(depth_img_shape):
    """Generate matrix of indeces for image in homogenous form

    Args:
        depth_img_shape: Shape of image to work with
    """
    indices = np.reshape(np.indices(np.flip(depth_img_shape)), (2, -1))
    indices_h = np.concatenate((indices, np.ones((1, indices.shape[1]))))
    return indices_h


def de_project_depth(points, k_inv, img):
    """Take points in the depth image from pixel space to 3D space,
    using the values in the image as the z-values to prevent scale
    ambiguity.

    Args:
        points: The points to bring through
        k_inv: The inverse intrinsics for the depth camera
        img: The actual image
    """
    return (k_inv@points) * \
        (np.reshape(img, (1, -1), order='F'))


def transform_depth_world2color_pixels(color_cam_matrix, world_d):
    return (color_cam_matrix @
            (np.concatenate((world_d, np.ones((1, world_d.shape[1]))))))


def project_depth_to_colorframe(px_color, color_img_shape):
    valid_px_color = px_color[2, :] != 0

    mapped_depth_img = np.zeros((color_img_shape[0], color_img_shape[1]))
    points = np.round(
        (px_color[:, valid_px_color] /
         px_color[2, valid_px_color])[:2, :].T[:, :2]).astype('uint16')
    valid = np.all((points[:, 0] > 0, points[:, 1] > 0,
                    points[:, 0] < mapped_depth_img.shape[1],
                    points[:, 1] < mapped_depth_img.shape[0]), axis=0)
    mapped_depth_img[points[valid, 1],
                     points[valid, 0]] = px_color[2, valid]
    return mapped_depth_img


def calc_depth_from_sparse_image(mapped_depth_img, poses, window):
    # we would kind of prefer min here, but don't want to end up
    # with zero values
    depths = [np.max(mapped_depth_img[max(0, pose[1]-window):
                                      min(pose[1]+window,
                                          mapped_depth_img.shape[0]),
                                      max(pose[0] - window, 0):
                                      min(pose[0]+window, mapped_depth_img.shape[1])]
                     ) for pose in np.round(poses).astype('uint16')]
    return depths


def deproject_colordepth(k_inv, poses, depths):
    poses_3d_c = (
        k_inv @
        np.concatenate((poses, np.ones((poses.shape[0], 1))), axis=1).T)*depths
    return poses_3d_c


def project_keypoints2depth(k_d, h_matrix_inv, poses_3d_c):
    poses_in_depth = (k_d @
                      (h_matrix_inv @
                       np.concatenate((poses_3d_c,
                                       np.ones((1, poses_3d_c.shape[1]))), axis=0))[:3, :])
    poses_in_depth = (poses_in_depth/poses_in_depth[2, :])[:2, :].T
    return poses_in_depth


def extract_depth(poses, depth_img, color_img_shape, transform_mats,
                  window):
    indices_h = generate_image_h_indices(depth_img.shape)
    window = int((window-1)/2)
    world_d = de_project_depth(indices_h, transform_mats['k_d_inv'], depth_img)
    px_color = transform_depth_world2color_pixels(
        transform_mats['color_cam_matrix'], world_d)
    mapped_depth_img = project_depth_to_colorframe(px_color, color_img_shape)
    depths = calc_depth_from_sparse_image(mapped_depth_img, poses, window)
    poses_3d_c = deproject_colordepth(transform_mats['k_c_inv'], poses, depths)
    poses_in_depth = project_keypoints2depth(
        transform_mats['k_d'], transform_mats['h_matrix_inv'], poses_3d_c)
    return(poses_3d_c, poses_in_depth)


def extract_depth_wrap(color_img_shape, transform_mats,
                       depth_img, keypoints, depth_time, color_time, idx):
    """Calculate 3D pose and pose in depth pixel

    This is meant to be safe to run in a multiprocessing pool.

    Prior to doing calculations, any values which are too close
    or too far away to be valid are set to zero. After all calculations,
    prior to returning, any keypoints which have a depth value that is
    too close or too far will be set to NaN.

    Args:
        color_img_shape: The shape of the color image that the
                         keypoints were detected in.
        transform_mats: A dictionary of various transformation
                        matricies.
        depth_img: The depth image with pixel values = millimeters.
        keypoints: The keypoints in the depth frame pixels.
        depth_time: The time that the depth image was captured.
        color_time: The time that the color image was captured.
        idx: IDX to operate at. This is just used to keep track
             of what job is being returned in certain async
             scenarios
    Returns:
        idx: The id that is being operated at
        poses_3d_c: The poses in 3D space in millimeters
        poses_in_depth: The poses in pixels in the depth frame
    """
    # pylint: disable= too-many-arguments

    if np.abs(
            depth_time -
            color_time
    ) > MAX_TIME_DISPARITY:
        return (idx, np.NaN, np.NaN)
        # keypoints3d_dset[idx, :, :] = np.NaN
        # keypoints_depth_dset[idx, :, :] = np.NaN
    depth_img[depth_img < MIN_VALID_DEPTH_MM] = 0
    depth_img[depth_img > MAX_VALID_DEPTH_MM] = 0
    poses_3d_c, poses_in_depth = \
        extract_depth(keypoints, depth_img,
                      color_img_shape, transform_mats, DEPTH_KERNEL_SIZE)
    invalid_indeces = np.logical_or(
        poses_3d_c[2, :] < MIN_VALID_DEPTH_MM,
        poses_3d_c[2, :] > MAX_VALID_DEPTH_MM
    )
    poses_3d_c[:, invalid_indeces] = np.nan
    poses_in_depth[invalid_indeces, :] = np.nan
    # keypoints3d_dset[idx, :, :] = poses_3d_c.T
    # keypoints_depth_dset[idx, :, :] = poses_in_depth
    return (idx, poses_3d_c.T, poses_in_depth)


def build_tranformations(r_cd, t_cd, k_d, k_c):
    """Build useful transformations and return in a dictionary.

    Args:
        r_cd: The rotation matrix between the color and depth imagers
        t_cd: The translation between the color and depth imagers
        k_d: The K matrix (intrinsics) for the depth imager
        k_c: The K matrix (intrinsics) for the color imager
    """
    h_matrix = np.eye(4)
    h_matrix[:3, :3] = r_cd.T
    h_matrix[:3, 3] = (t_cd).flatten()
    k_c_padded = np.concatenate((k_c, np.zeros((3, 1))), axis=1)
    color_cam_matrix = k_c_padded@h_matrix
    k_d_inv = np.linalg.inv(k_d)
    k_c_inv = np.linalg.inv(k_c)
    h_matrix_inv = h_matrix.copy()
    h_matrix_inv[:3, :3] = np.linalg.inv(h_matrix[:3, :3])
    h_matrix_inv[:3, 3] = -h_matrix_inv[:3, :3]@h_matrix[:3, 3]

    transform_mats = dict()
    transform_mats['h_matrix'] = h_matrix
    transform_mats['k_c_padded'] = k_c_padded
    transform_mats['color_cam_matrix'] = color_cam_matrix
    transform_mats['k_d_inv'] = k_d_inv
    transform_mats['k_c_inv'] = k_c_inv
    transform_mats['h_matrix_inv'] = h_matrix_inv
    transform_mats['k_d'] = k_d
    transform_mats['k_c'] = k_c

    return transform_mats


def generate_keypoints_3d_dset(hdf5_out, pose_dset_root, keypoints_dset_name):
    """Get the dataset to store 3D keypoints in.

    This dataset will hold the keypoints in 3D in millimeters.

    If the dataset already exists, just return it. If not, create it.

    Args:
        hdf5_out: The HDF5 file to put the keypoints in
        pose_dset_root: The root of the keypoint detections to work on
        keypoints_dset_name: The name of the dataset which holds the
                             keypoints to work on.
    """
    keypoints3d_dset_name = f'{pose_dset_root}/3d-realsense-raw'
    if keypoints3d_dset_name not in hdf5_out:
        keypoints_shape = hdf5_out[keypoints_dset_name].shape
        keypoints3d_dset = hdf5_out.create_dataset(
            keypoints3d_dset_name,
            (keypoints_shape[0], keypoints_shape[1], 3),
            dtype=np.float32)
    else:
        keypoints3d_dset = hdf5_out[keypoints3d_dset_name]
        print('You might be running the Stereo depth extraction twice')

    keypoints3d_dset.attrs['desc'] = \
        'Keypoints in 3D coordinates using the raw depth from the ' +\
        'realsense camera, indexed at keypoints. Depth is used to ' +\
        'provide x, y, and z in metric space (meters)'
    return keypoints3d_dset


def generate_keypoints_depth_dset(hdf5_out, pose_dset_root, keypoints_dset_name):
    """Get the dataset to store keypoints in the depth frame in.

    This dataset will hold the pixel coordinates for the keypoints, in the depth
    image.

    If the dataset already exists, just return it. If not, create it.

    Args:
        hdf5_out: The HDF5 file to put the keypoints in
        pose_dset_root: The root of the keypoint detections to work on
        keypoints_dset_name: The name of the dataset which holds the
                             keypoints to work on.
    """
    keypoints_depth_dset_name = f'{pose_dset_root}/depth'
    if keypoints_depth_dset_name not in hdf5_out:
        keypoints_shape = hdf5_out[keypoints_dset_name].shape
        keypoints_depth_dset = hdf5_out.create_dataset(
            keypoints_depth_dset_name,
            (keypoints_shape[0], keypoints_shape[1], 2),
            dtype=np.float32)
    else:
        keypoints_depth_dset = hdf5_out[keypoints_depth_dset_name]
        print('You might be running the Stereo depth extraction twice')

    keypoints_depth_dset.attrs['desc'] =\
        'Keypoints in the depth image frame space (pixels)'
    return keypoints_depth_dset


def get_extinsics(hdf5_in, color_dset_name, color_time_dset_name, transforms):
    """Get the camera extrinsics (color to depth camera)

    Will attempt to find the extrinsics in the hdf5 file. If not availalbe,
    then will find the best transform in the transforms dictionary.

    Args:
        hdf5_in: The HDF5 file wwith the full video feeds
        color_dset_name: Name of the color image dataset
        color_time_dset_name: Name of the dataset with the times the color images
                              were captured
        transforms: The backup dictionary of known extrinsics between the cameras
    """
    if ('depth_to_color-rotation' in hdf5_in[color_dset_name].attrs and
            'depth_to_color-translation' in hdf5_in[color_dset_name].attrs):
        r_cd_raw = hdf5_in[color_dset_name].attrs['depth_to_color-rotation']
        t_cd_raw = hdf5_in[color_dset_name].attrs['depth_to_color-translation']
    else:
        # transforms is already the
        time_target = hdf5_in[color_time_dset_name][0] + \
            (hdf5_in[color_time_dset_name][-1] -
             hdf5_in[color_time_dset_name][0])/2
        transform_idx = np.argmin(
            np.abs(np.asarray([float(v) for v in transforms.keys()])-time_target))
        best_transform = transforms[[*transforms][transform_idx]]
        r_cd_raw = best_transform['rotation']
        t_cd_raw = best_transform['translation']
    r_cd = np.asarray(r_cd_raw).reshape(3, 3)
    t_cd = np.asarray(t_cd_raw).reshape(3, 1)*1000  # get to millimeters
    return(t_cd, r_cd)


def get_intrinsics(hdf5_in, color_dset_name, depth_dset_name):
    """Get intrinsics for the depth and color cameras

    Args:
        hdf5_in: The HDF5 file wwith the full video feeds
        color_dset_name: Name of the color image dataset
        depth_dset_name: Neme of the depth image dataset
    """
    k_c = hdf5_in[color_dset_name].attrs['K'].reshape(3, 3)
    k_d = hdf5_in[depth_dset_name].attrs['K'].reshape(3, 3)
    return(k_c, k_d)


def add_stereo_depth(hdf5_in, hdf5_out, cam_root, pose_dset_root, transforms=None):
    """Add stereo depth to hdf5 out file given keypoints and depth images.

    It is necessary to know the camera intrinsics and extrinsics. This can be pulled out
    by the attributes in the hdf5 files if the data was present (isn't always there).
    If not, then this can be pulled out of a transforms file which gets the transforms
    from other bag files. This transforms file is generated by `get_transforms/run`.
    This json file should be opened and this function should only be passed the sub
    dictionary indexed by the source and camera.

    This uses data from both hdf5_in and out but only writes to out.

    This requires that a previous system has already discovered and saved the best
    matching depth image to the color (done by `convert_to_hdf5/src/convert_to_hdf5.py:match_depth`)
    Any depth matches which are outside of MAX_TIME_DISPARITY will be saved as nan.

    Any depth data that falls outside of [ MIN_VALID_DEPTH_METERS, MAX_VALID_DEPTH_METERS ] will be
    saved as nan.

    Creates two new datasets a 3dkeypoints one which is in metric 3d space and a keypoints-depth
    dataset which is the keypoints found by the algorithm mapped into the depth image's image
    space. Note however that the depth image keypoints will be aligned with the color data's time.

    Args:
        hdf5_in: The opened hdf5 file with the full data with video.
        hdf5_out: The opened hdf5 file to put data into and which already has
                  keypoints extracted.
        cam_root: The name of the camera to use ('lower' or 'upper')
        transforms: The dictionary with keys that are timestamps (sec since epoch) with fields
                    'rotation': 9 element list (represents 3x3 rot array) and 'translation': 3
                    element list representing translation.
    """
    # pylint: disable= too-many-locals
    depth_match_dset_name = f'{cam_root}/color/matched_depth_index'
    color_dset_name = f'{cam_root}/color/data'
    color_time_dset_name = f'{cam_root}/color/time'
    depth_dset_name = f"{cam_root}/depth/data"
    depth_time_dset_name = f'{cam_root}/depth/time'

    keypoints_dset_name = f'{pose_dset_root}/color'

    keypoints3d_dset = generate_keypoints_3d_dset(
        hdf5_out, pose_dset_root, keypoints_dset_name)

    keypoints_depth_dset = generate_keypoints_depth_dset(
        hdf5_out, pose_dset_root, keypoints_dset_name)

    t_cd, r_cd = get_extinsics(
        hdf5_in, color_dset_name, color_time_dset_name, transforms)

    k_c, k_d = get_intrinsics(hdf5_in, color_dset_name, depth_dset_name)
    transform_mats = build_tranformations(r_cd, t_cd, k_d, k_c)
    color_img_shape = hdf5_in[color_dset_name][0].shape

    num_frames = hdf5_in[color_dset_name].shape[0]
    matched_index_l = [None]*num_frames
    keypoints_l = [None] * num_frames
    depth_time_l = [None]*num_frames
    color_time_l = [None]*num_frames
    depth_img_l = [None]*num_frames
    for idx in range(num_frames):
        matched_index_l[idx] = hdf5_in[depth_match_dset_name][idx]
        keypoints_l[idx] = hdf5_out[keypoints_dset_name][idx]
        depth_time_l[idx] = hdf5_in[depth_time_dset_name][matched_index_l[idx]]
        color_time_l[idx] = hdf5_in[color_time_dset_name][idx]
        depth_img_l[idx] = hdf5_in[depth_dset_name][matched_index_l[idx]]
    print('done with setup, starting multiprocessing run')
    bound_func = partial(
        extract_depth_wrap, color_img_shape, transform_mats)
    args = zip(depth_img_l, keypoints_l, depth_time_l,
               color_time_l, range(num_frames))
    with multiprocessing.Pool() as pool:
        results = pool.starmap(bound_func, tqdm.tqdm(
            args, total=num_frames), chunksize=10)
    for result in results:
        idx = result[0]
        keypoints3d_dset[idx, :, :] = result[1]
        keypoints_depth_dset[idx, :, :] = result[2]
