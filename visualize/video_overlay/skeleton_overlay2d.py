"""Module to overlay 2D skeleton"""
#!/usr/bin/env python3

import pathlib
import h5py
from tqdm import trange
import cv2
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
from pose.src.joints import openpose_joint_pairs, openpose_upper_body_joints, mphands_joint_pairs
from common import color
from common import img_overlays
from common import plot_helpers
from common.realsense_params import MIN_VALID_DEPTH_METERS
from common.realsense_params import MAX_VALID_DEPTH_METERS
from common.tracking_params import MIN_CONFIDENCE


MIN_VALID_DEPTH_MM = MIN_VALID_DEPTH_METERS*1000
MAX_VALID_DEPTH_MM = MAX_VALID_DEPTH_METERS*1000


def plot_joints(img, keypoints, confidence, joint_idx):
    """Plot joint positions on image

    plotting is done inplace

    Args:
        img: The image to plot on
        keypoints: The kypoints to plot
        confidence: The confidence of the detections
        joint_idx: The joint indeces to plot
    """
    for joint in joint_idx:
        if ((not np.any(np.isnan(keypoints[joint][0:2]))) and
                confidence[joint] > MIN_CONFIDENCE):
            x_pos = int(keypoints[joint][0])
            y_pos = int(keypoints[joint][1])
            cv2.circle(img, (x_pos, y_pos), 10,
                       color.color_scale(confidence[joint], 0, 1), 8)


def plot_limbs(img, keypoints, confidence, joint_pairs):
    """plot limb connections

    does plotting in place

    Args:
        img: Image
        keypoints: Keypoints to plot between
        confidence: Confidence of those keypoints
        joint_pairs: The joint pairs to plot
    """
    for joint in joint_pairs:
        if ((not np.any(np.isnan([keypoints[joint[jidx]][0:2] for jidx in (0, 1)])))
                and np.all([confidence[joint[jidx]] > MIN_CONFIDENCE for jidx in (0, 1)])):
            x_0 = int(keypoints[joint[0]][0])
            y_0 = int(keypoints[joint[0]][1])

            x_1 = int(keypoints[joint[1]][0])
            y_1 = int(keypoints[joint[1]][1])

            cv2.line(img, (x_0, y_0), (x_1, y_1), color.color_scale(
                confidence[joint[0]] + confidence[joint[1]], 0, 1))


def plot_overhead(img, keypoints_3d, scaling_params, confidence, joint_pairs, pos):
    """Plot an overhead skeletal view

    plotting is done inplace.

    Args:
        img: Image to plot on
        keypoints_3d: The 3d keypoints
        scaling_params: list/tuple with scaling parameters:
                        max_range, x_percentiles, y_percentiles, z_percentiles
        confidence: Confidence of detections
        pos: tuple with 4 elements: (the pixel width to plot over,
             the pixel height to plot over, the x position for the left side of the plotting,
             the y position for the top of the plotting)
    """
    #pylint: disable=too-many-arguments
    plot_scale = (np.min([pos[0], pos[1]])/2)/(1.2*scaling_params[0])
    for limb in joint_pairs:
        if ((not np.any(np.isnan(
            [keypoints_3d[limb[jidx]][0:2] for jidx in (0, 1)]
        )))
                and np.all(
                    [confidence[limb[jidx]] >
                        MIN_CONFIDENCE for jidx in (0, 1)]
        )):
            x_0 = int(pos[0]/2 + pos[2] +
                      (keypoints_3d[limb[0]][0]-scaling_params[1][1])*plot_scale)
            z_0 = int(pos[3] + pos[1]/2 -
                      (keypoints_3d[limb[0]][2]-scaling_params[3][1])*plot_scale)

            x_1 = int(pos[0]/2 +
                      (keypoints_3d[limb[1]][0]-scaling_params[1][1])*plot_scale)
            z_1 = int(pos[3] + pos[1]/2 -
                      (keypoints_3d[limb[1]][2]-scaling_params[3][1])*plot_scale)

            cv2.line(img, (x_0, z_0), (x_1, z_1), color.color_scale(
                confidence[limb[0]] + confidence[limb[1]], 0, 3))

    img_overlays.draw_text(
        img, 'view: top (synced to depth)', pos=(pos[2]+20, pos[3]+3))


def visualize_depth(depth_img):
    """Make depth image easy to visualize

    Depth data can be hard to visualize, especially in an opencv
    generated video, where the bitdepth is very small per channel

    Want to use a colormap to fully utilize the data in the image.
    depth images are stored as 16 bit ints (realistically use
    2500 steps of data). 8 int grayscale only has 255 steps.
    By using all 3 channels, we get more than enough fidelity.
    colormap outputs a float 0-1, so take to 16 bit and make integer.

    Args:
        depth_img: The depth image to work on. Should be a single channel
                   16bit int matrix

    Return: The depth image in color, ready to visualize
    """
    # by default vridis only has 0xFF steps, so lets take that out
    # to a full 16 bits.
    colormap = mcolors.LinearSegmentedColormap.from_list(
        "", plt.cm.viridis.colors, N=0xFFFF)

    depth_img = np.uint16(
        np.clip(depth_img, MIN_VALID_DEPTH_MM, MAX_VALID_DEPTH_MM))
    depth_img = plot_helpers.stretch_histogram(depth_img)
    # The fourth channel is alpha, so get rid of that
    depth_img = (colormap(depth_img) * 0xFFFF).astype(np.uint16)[:, :, :3]
    depth_img = cv2.cvtColor(depth_img, cv2.COLOR_RGB2BGR)
    # convert from 16 to 8 bit
    depth_img = cv2.convertScaleAbs(depth_img, alpha=(0xFF/0xFFFF))
    return depth_img


def plot_joints_limbs(img, keypoints, confidence, joints, limbs):
    """Plot the joints and limbs on an image

    Args:
        img: Image to plot on
        keypoints: Keypoints to plot
        confidence: Confidence of detection, for each keypoint
        joints: The joint indeces to plot
        limbs: The limb definitions as a list of tuples defining
               the indeces to connect
    """
    plot_joints(img, keypoints, confidence, joints)
    plot_limbs(img, keypoints, confidence, limbs)


def plot_openpose_color(hdf5_tracking, dset_names, algorithm, idx, color_img):
    """Plot openpose joints and limbs on color image

    Args:
        hdf5_tracking: hdf5 tracking file, open
        dset_names: Names of datasets in hdf5 tracking
        algorithm: Algorithm to plot from
        idx: Index for the image
        color_img: The image to plot on
    """
    color_keypoints = hdf5_tracking[dset_names[algorithm]
                                    ['keypoints_color']][idx]
    confidence = hdf5_tracking[dset_names[algorithm]
                               ['confidence']][idx]
    joints = openpose_upper_body_joints()
    limbs = openpose_joint_pairs()
    plot_joints_limbs(color_img, color_keypoints,
                      confidence, joints, limbs)


def plot_openpose_depth(hdf5_tracking, dset_names, algorithm, idx, depth_img):
    """Plot openpose joints and limbs on depth image

    Args:
        hdf5_tracking: hdf5 tracking file, open
        dset_names: Names of datasets in hdf5 tracking
        algorithm: Algorithm to plot from
        idx: Index for the image
        depth_img: The image to plot on
    """
    depth_keypoints = hdf5_tracking[dset_names[algorithm]
                                    ['keypoints_depth']][idx]
    confidence = hdf5_tracking[dset_names[algorithm]
                               ['confidence']][idx]
    joints = openpose_upper_body_joints()
    limbs = openpose_joint_pairs()
    plot_joints_limbs(depth_img, depth_keypoints,
                      confidence, joints, limbs)


def plot_mphands_color(hdf5_tracking, dset_names, algorithm, idx, color_img):
    """Plot media pose hands joints and limbs on color image

    Args:
        hdf5_tracking: hdf5 tracking file, open
        dset_names: Names of datasets in hdf5 tracking
        algorithm: Algorithm to plot from
        idx: Index for the image
        color_img: The image to plot on
    """
    for hand in ('left', 'right'):
        color_keypoints = hdf5_tracking[dset_names[algorithm]
                                        [hand]['keypoints_color']][idx]
        confidence = np.repeat(hdf5_tracking[dset_names[algorithm]
                                             [hand]['confidence']][idx],
                               len(color_keypoints))
        joints = np.arange(0, 21)
        limbs = mphands_joint_pairs()
        plot_joints_limbs(color_img, color_keypoints,
                          confidence, joints, limbs)


def plot_mphands_depth(hdf5_tracking, dset_names, algorithm, idx, depth_img):
    """Plot media pose hands joints and limbs on depth image

    Args:
        hdf5_tracking: hdf5 tracking file, open
        dset_names: Names of datasets in hdf5 tracking
        algorithm: Algorithm to plot from
        idx: Index for the image
        depth_img: The image to plot on
    """
    for hand in ('left', 'right'):
        depth_keypoints = hdf5_tracking[dset_names[algorithm]
                                        [hand]['keypoints_depth']][idx]
        confidence = np.repeat(hdf5_tracking[dset_names[algorithm]
                                             [hand]['confidence']][idx],
                               len(depth_keypoints))
        joints = np.arange(0, 21)
        limbs = mphands_joint_pairs()
        plot_joints_limbs(depth_img, depth_keypoints,
                          confidence, joints, limbs)


def plot_overhead_openpose(
        hdf5_tracking, dset_names, algorithm, idx, img, scaling_params, pos):
    """Plot an overhead view of openpose skeleton

    Args:
        hdf5_tracking: hdf5 tracking file, open
        dset_names: Names of datasets in hdf5 tracking
        algorithm: Algorithm to plot from
        idx: Index for the image
        img: The image to plot on
        scaling_params: list/tuple with scaling parameters:
                        max_range, x_percentiles, y_percentiles, z_percentiles
        pos: tuple with 4 elements: (the pixel width to plot over,
             the pixel height to plot over, the x position for the left side of the plotting,
             the y position for the top of the plotting)
    """
    #pylint: disable=too-many-arguments
    confidence = hdf5_tracking[dset_names[algorithm]['confidence']][idx]
    keypoints_3d = hdf5_tracking[dset_names[algorithm]['3dkeypoints']][idx]
    plot_overhead(img, keypoints_3d,
                  scaling_params, confidence, openpose_joint_pairs(),
                  pos)


def plot_overhead_mphands(hdf5_tracking, dset_names, algorithm, idx,
                          img, scaling_params, pos):
    """Plot an overhead view of mediapipe hands skeleton

    Args:
        hdf5_tracking: hdf5 tracking file, open
        dset_names: Names of datasets in hdf5 tracking
        algorithm: Algorithm to plot from
        idx: Index for the image
        img: The image to plot on
        scaling_params: list/tuple with scaling parameters:
                        max_range, x_percentiles, y_percentiles, z_percentiles
        pos: tuple with 4 elements: (the pixel width to plot over,
             the pixel height to plot over, the x position for the left side of the plotting,
             the y position for the top of the plotting)
    """
    #pylint: disable=too-many-arguments
    for hand in ('left', 'right'):
        keypoints_3d = hdf5_tracking[dset_names[algorithm]
                                     [hand]['3dkeypoints']][idx]
        confidence = np.repeat(hdf5_tracking[dset_names[algorithm]
                                             [hand]['confidence']][idx],
                               len(keypoints_3d))
        plot_overhead(
            img, keypoints_3d,
            scaling_params, confidence, mphands_joint_pairs(),
            pos)


def calculate_scaling_params_all(hdf5_tracking, dset_names, algorithms):
    """Calculate the scaling parameters across all of the algorithms
    passed in

    Args:
        hdf5_tracking: hdf5 tracking file, open
        dset_names: Names of datasets in hdf5 tracking
        algorithm: Algorithms to work with
    """
    dsets = np.zeros((0, 3))
    for algorithm in algorithms:
        if 'openpose:25' in algorithm:
            dsets = np.append(
                dsets,
                np.reshape(
                    hdf5_tracking[dset_names[algorithm]["3dkeypoints"]],
                    (-1, 3)), axis=0)
        elif 'mp-hands' in algorithm:
            dsets = np.append(
                dsets,
                np.reshape(
                    hdf5_tracking[dset_names[algorithm]
                                  ['left']["3dkeypoints"]],
                    (-1, 3)), axis=0)
            dsets = np.append(
                dsets,
                np.reshape(
                    hdf5_tracking[dset_names[algorithm]
                                  ['right']["3dkeypoints"]],
                    (-1, 3)), axis=0)
        else:
            raise NotImplementedError(
                f'Scaling params for algorithm ({algorithm}) not implemented')

    return plot_helpers.calculate_data_range(dsets)


def overlay_2d_skeleton(directory, cam, dset_names, algorithms):
    """Overlay 2D skeleton on color and depth images and visualize from
    overhead.

    Requires the full_data-novid.hdf5 and full_data-vid.hdf5 files.

    Args:
        directory: The directory with the two hdf5 files
        cam: The camera to work with (lower, upper)
        dset_names: The names of the datasets in the hdf file to process on
        algorithms: pose algorithms to plot from
    """
    directory = pathlib.Path(directory)
    hdf5_video = h5py.File(directory/'full_data-vid.hdf5')
    hdf5_tracking = h5py.File(directory/'full_data-novid.hdf5')

    scaling_params = calculate_scaling_params_all(
        hdf5_tracking, dset_names, algorithms)

    # note, video writer can only take unsigned 8 bit integer images
    video_writer = cv2.VideoWriter(
        str(directory/f'viz-{cam}-2dSkeleton.mkv'),
        cv2.VideoWriter_fourcc(*'mp4v'), 30, (1920, 1080 + 720))

    for idx in trange(hdf5_video[dset_names['color_dset']].shape[0]):
        depth_idx = hdf5_video[dset_names['depth_mapping_dset']][idx]
        color_img = hdf5_video[dset_names['color_dset']][idx]
        depth_img = visualize_depth(
            hdf5_video[dset_names['depth_dset']][depth_idx])

        img_overlays.draw_cam_info(
            color_img, idx, hdf5_tracking[dset_names['time_color']][idx], cam)
        img_overlays.draw_cam_info(
            depth_img, depth_idx, hdf5_tracking[dset_names['time_depth']][depth_idx], cam, "depth")

        for algorithm in algorithms:
            if 'openpose:25' in algorithm:
                plot_openpose_color(
                    hdf5_tracking, dset_names, algorithm, idx, color_img)
                plot_openpose_depth(
                    hdf5_tracking, dset_names, algorithm, idx, depth_img)
            elif 'mp-hands' in algorithm:
                plot_mphands_color(hdf5_tracking, dset_names,
                                   algorithm, idx, color_img)
                plot_mphands_depth(hdf5_tracking, dset_names,
                                   algorithm, idx, depth_img)
            else:
                raise NotImplementedError(
                    f'Plotting on color and depth not implemented for algorithm ({algorithm})')

        depth_img = cv2.copyMakeBorder(
            depth_img, 0, 0, int((color_img.shape[1]-depth_img.shape[1])),
            0, cv2.BORDER_CONSTANT)
        img = cv2.vconcat([color_img, depth_img])

        # add top skeleton view
        # w=1920-1280 = 640
        # h=720
        # overhead, so draw x and z
        overhead_pos = (640, 720, 0, 1080)
        for algorithm in algorithms:
            if 'openpose:25' in algorithm:
                plot_overhead_openpose(
                    hdf5_tracking, dset_names, algorithm, idx, img, scaling_params, overhead_pos)
            elif 'mp-hands' in algorithm:
                plot_overhead_mphands(
                    hdf5_tracking, dset_names, algorithm, idx, img, scaling_params, overhead_pos)

        video_writer.write(img)

    video_writer.release()
    hdf5_video.close()
    hdf5_tracking.close()
