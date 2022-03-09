"""Module to overlay 2D skeleton"""
#!/usr/bin/env python3

import pathlib
import h5py
from tqdm import trange
import cv2
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
from pose.src.joints import openpose_joints
from common import color
from common import img_overlays
from common import plot_helpers
from common.realsense_params import MIN_VALID_DEPTH_METERS
from common.realsense_params import MAX_VALID_DEPTH_METERS
from common.tracking_params import MIN_CONFIDENCE

OPENPOSE_JNTS = openpose_joints()
# Joints listed here: https://github.com/CMU-Perceptual-Computing-Lab/openpose/
# blob/master/doc/02_output.md#keypoints-in-cpython
PAIRS = [
    (OPENPOSE_JNTS.index('UpperNeck'), OPENPOSE_JNTS.index('LShoulder')),
    (OPENPOSE_JNTS.index('UpperNeck'), OPENPOSE_JNTS.index('RShoulder')),
    (OPENPOSE_JNTS.index('LShoulder'), OPENPOSE_JNTS.index('LElbow')),
    (OPENPOSE_JNTS.index('RShoulder'), OPENPOSE_JNTS.index('RElbow')),
    (OPENPOSE_JNTS.index('LElbow'),    OPENPOSE_JNTS.index('LWrist')),
    (OPENPOSE_JNTS.index('RElbow'),    OPENPOSE_JNTS.index('RWrist')),
    (OPENPOSE_JNTS.index('UpperNeck'), OPENPOSE_JNTS.index('LHip')),
    (OPENPOSE_JNTS.index('UpperNeck'), OPENPOSE_JNTS.index('RHip'))
]

OPENPOSE_JOINTS = [openpose_joints().index(key) for key in [
    "Nose", "LEye", "REye", "LEar", "REar", "LShoulder", "RShoulder",
            "LElbow", "RElbow", "LWrist", "RWrist", "LHip", "RHip", "UpperNeck",
            "HeadTop"
]
]

MIN_VALID_DEPTH_MM = MIN_VALID_DEPTH_METERS*1000
MAX_VALID_DEPTH_MM = MAX_VALID_DEPTH_METERS*1000


def stretch_histogram(img):
    """Stretch histogram of 16 bit single channel image

    https://docs.opencv.org/4.x/d5/daf/tutorial_py_histogram_equalization.html
    this does the same thing as cv2.equalizeHist, but for 16 bit images
    (as opposed to 8 bit)

    Args:
        img: The image to stretch

    Returns: the image, stretched
    """
    hist, _ = np.histogram(img.flatten(), 0xFFFF+1, [0, 0xFFFF+1])
    cdf = hist.cumsum()
    # cdf_normalized = cdf * float(hist.max()) / cdf.max()
    cdf_m = np.ma.masked_equal(cdf, 0)
    cdf_m = (cdf_m - cdf_m.min())*0xFFFF/(cdf_m.max()-cdf_m.min())
    cdf = np.ma.filled(cdf_m, 0).astype('uint16')
    return cdf[img]


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


def plot_limbs(img, keypoints, confidence):
    """plot limb connections

    does plotting in place

    Args:
        img: Image
        keypoints: Keypoints to plot between
        confidence: Confidence of those keypoints
    """
    for joint in PAIRS:
        if ((not np.any(np.isnan([keypoints[joint[jidx]][0:2] for jidx in (0, 1)])))
                and np.all([confidence[joint[jidx]] > MIN_CONFIDENCE for jidx in (0, 1)])):
            x_0 = int(keypoints[joint[0]][0])
            y_0 = int(keypoints[joint[0]][1])

            x_1 = int(keypoints[joint[1]][0])
            y_1 = int(keypoints[joint[1]][1])

            cv2.line(img, (x_0, y_0), (x_1, y_1), color.color_scale(
                confidence[joint[0]] + confidence[joint[1]], 0, 1))


def plot_overhead(img, keypoints_3d, scaling_params, confidence, pos):
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
    plot_scale = (np.min([pos[0], pos[1]])/2)/1.2*scaling_params[0]
    for limb in PAIRS:
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
    depth_img = stretch_histogram(depth_img)
    # The fourth channel is alpha, so get rid of that
    depth_img = (colormap(depth_img) * 0xFFFF).astype(np.uint16)[:, :, :3]
    depth_img = cv2.cvtColor(depth_img, cv2.COLOR_RGB2BGR)
    # convert from 16 to 8 bit
    depth_img = cv2.convertScaleAbs(depth_img, alpha=(0xFF/0xFFFF))
    return depth_img


def overlay_2d_skeleton(directory, cam, dset_names):
    """Overlay 2D skeleton on color and depth images and visualize from
    overhead.

    Requires the full_data-novid.hdf5 and full_data-vid.hdf5 files.

    Args:
        directory: The directory with the two hdf5 files
        cam: The camera to work with (lower, upper)
        dset_names: The names of the datasets in the hdf file to process on
    """
    directory = pathlib.Path(directory)
    hdf5_video = h5py.File(directory/'full_data-vid.hdf5')
    hdf5_tracking = h5py.File(directory/'full_data-novid.hdf5')

    scaling_params = plot_helpers.calculate_data_range(
        hdf5_tracking[dset_names["3dkeypoints"]])

    # note, video writer can only take unsigned 8 bit integer images
    video_writer = cv2.VideoWriter(
        str(directory/f'viz-{cam}-2dSkeleton.mkv'),
        cv2.VideoWriter_fourcc(*'mp4v'), 30, (1920, 1080 + 720))

    for idx in trange(hdf5_video[dset_names['color_dset']].shape[0]):
        depth_idx = hdf5_video[dset_names['depth_mapping_dset']][idx]
        color_img = hdf5_video[dset_names['color_dset']][idx]
        depth_img = visualize_depth(
            hdf5_video[dset_names['depth_dset']][depth_idx])

        confidence = hdf5_tracking[dset_names['confidence']][idx]

        img_overlays.draw_cam_info(
            color_img, idx, hdf5_tracking[dset_names['time_color']][idx], cam)
        img_overlays.draw_cam_info(
            depth_img, depth_idx, hdf5_tracking[dset_names['time_depth']][depth_idx], cam, "depth")

        for img, keypoints in zip((color_img, depth_img),
                                  (hdf5_tracking[dset_names['keypoints_color']][idx],
                                   hdf5_tracking[dset_names['keypoints_depth']][idx])):
            plot_joints(img, keypoints, confidence, OPENPOSE_JOINTS)
            plot_limbs(img, keypoints, confidence)

        w_padding = int((color_img.shape[1]-depth_img.shape[1]))
        depth_img = cv2.copyMakeBorder(
            depth_img, 0, 0, w_padding, 0, cv2.BORDER_CONSTANT)
        img = cv2.vconcat([color_img, depth_img])

        # add top skeleton view
        # w=1920-1280 = 640
        # h=720
        # overhead, so draw x and z
        plot_overhead(img, hdf5_tracking[dset_names['3dkeypoints']][idx],
                      scaling_params, confidence, (640, 720, 0, 1080))

        video_writer.write(img)

    video_writer.release()
    hdf5_video.close()
    hdf5_tracking.close()
