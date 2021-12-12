#!/usr/bin/env python3

"""A module for overlaying data and text onto images"""

import math
import sys
import numpy as np
import h5py
import cv2
from tqdm import trange
from common import img_overlays

# from: https://stackoverflow.com/a/65146731/5274985


def color_scale(mag, cmin, cmax):
    """ Return a tuple of floats between 0 and 1 for R, G, and B. """
    # Normalize to 0-1
    try:
        scale = float(mag-cmin)/(cmax-cmin)
    except ZeroDivisionError:
        scale = 0.5  # cmax == cmin
    blue = min((max((4*(0.75-scale), 0.)), 1.))
    red = min((max((4*(scale-0.25), 0.)), 1.))
    green = min((max((4*math.fabs(scale-0.5)-1., 0.)), 1.))
    return int(red*255), int(green*255), int(blue*255)


def overlay(file_stub, cam):  # pylint: disable=too-many-locals
    """overlay data from hdf5 file onto images from hdf file.

    Requires both the no video and video hdf5 files.

    Args:
        file_stub: The common file stub for the two hdf5 files
        cam: The camera to use (upper or lower)
    """
    hdf5_video = h5py.File(file_stub+'.hdf5')
    hdf5_tracking = h5py.File(file_stub+'-novid.hdf5')

    dset = 'vid/color/data/{}/data'.format(cam)
    video_writer = cv2.VideoWriter(
        file_stub+'-'+cam+'-wrist.avi', cv2.VideoWriter_fourcc(*'MJPG'), 30, (1920, 1080))
    t_vals = np.zeros(500, dtype=np.float64)
    x_l_vals = np.ones(500, dtype=np.float64)
    y_l_vals = np.ones(500, dtype=np.float64)
    x_r_vals = np.ones(500, dtype=np.float64)
    y_r_vals = np.ones(500, dtype=np.float64)
    for idx in trange(hdf5_video[dset].shape[0]):
        img = hdf5_video[dset][idx]
        keypoints = hdf5_tracking[dset+'-keypoints'][idx]
        confidence = hdf5_tracking[dset+'-confidence'][idx]
        sec = hdf5_tracking['vid/color/data/{}/secs'.format(cam)][idx]
        nsec = hdf5_tracking['vid/color/data/{}/nsecs'.format(cam)][idx]
        time = sec+nsec*1e-9

        img_overlays.draw_text(img, 'frame: {}'.format(idx), pos=(100, 3))
        img_overlays.draw_text(img, 'time: {:.2f}'.format(time), pos=(500, 3))
        img_overlays.draw_text(
            img, 'view: {} realsense'.format(cam), pos=(900, 3))

        # Joints listed here: https://github.com/CMU-Perceptual-Computing-Lab/openpose/
        # blob/master/doc/02_output.md#keypoints-in-cpython
        for joint in (4, 7):
            x = int(keypoints[joint][0])  # pylint: disable=invalid-name
            y = int(keypoints[joint][1])  # pylint: disable=invalid-name
            cv2.circle(img, (x, y), 20,
                       color_scale(confidence[4], 0, 1), 8)

        # TODO: get this all in a clean loop
        # TODO: plot on top of each other
        t_vals = np.roll(t_vals, 1)
        x_l_vals = np.roll(x_l_vals, 1)
        y_l_vals = np.roll(y_l_vals, 1)
        x_r_vals = np.roll(x_r_vals, 1)
        y_r_vals = np.roll(y_r_vals, 1)
        t_vals[0] = time
        x_l_vals[0] = keypoints[4][0]
        y_l_vals[0] = keypoints[4][1]
        x_r_vals[0] = keypoints[7][0]
        y_r_vals[0] = keypoints[7][1]
        if idx == 0:
            x_l_vals[-1] = keypoints[4][0]
            y_l_vals[-1] = keypoints[4][1]
            x_r_vals[-1] = keypoints[7][0]
            y_r_vals[-1] = keypoints[7][1]
            t_vals[-1] = time

        adj_time = t_vals - time
        in_range = adj_time > -15
        scaled_time = adj_time * 30 + 450

        cv2.polylines(img,
                      [np.int32(
                          np.transpose(
                              (scaled_time[in_range],
                               200+(.2*x_l_vals[in_range]))))],
                      isClosed=False, color=(250, 0, 0), thickness=4)
        cv2.polylines(img,
                      [np.int32(np.transpose(
                          (scaled_time[in_range], 400+(.2*y_l_vals[in_range]))))],
                      isClosed=False, color=(250, 250, 0), thickness=4)
        cv2.polylines(img,
                      [np.int32(np.transpose(
                          (scaled_time[in_range], 600+(.2*x_r_vals[in_range]))))],
                      isClosed=False, color=(250, 0, 250), thickness=4)
        cv2.polylines(img,
                      [np.int32(np.transpose(
                          (scaled_time[in_range], 800+(.2*y_r_vals[in_range]))))],
                      isClosed=False, color=(0, 250, 0), thickness=4)

        # img[0:300, 10:50] = x_l_plot.render()
        # img[0:300, 55:95] = y_l_plot.render()
        # img[0:300, 100:140] = x_r_plot.render()
        # img[0:300, 150:190] = y_r_plot.render()

        # cv2.imshow('im', img)
        # cv2.waitKey(1)
        video_writer.write(img)
    video_writer.release()
    hdf5_video.close()
    hdf5_tracking.close()


# Expect one argument of form: <file stub> which will be used to access
# <file stub>.hdf5 and <filestub>-novid.hdf5 and create <file stub>-wrists.avi
if __name__ == '__main__':
    overlay(sys.argv[1], sys.argv[2])
