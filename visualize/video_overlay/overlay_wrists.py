#!/usr/bin/env python3

import h5py
from tqdm import tqdm, trange
import cv2
import math
import sys

# from: https://www.oreilly.com/library/view/python-cookbook/0596001673/ch09s11.html


def colorScale(mag, cmin, cmax):
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


def overlay(file_stub):
    hdf5_video = h5py.File(file_stub+'.hdf5')
    hdf5_tracking = h5py.File(file_stub+'-novid.hdf5')

    for cam in tqdm(('upper', 'lower')):
        dset = 'vid/color/data/{}/data'.format(cam)
        video_writer = cv2.VideoWriter(
            file_stub+cam+'-wrist.avi', cv2.VideoWriter_fourcc(*'MJPG'), 10, (1920, 1080))
        for idx in trange(hdf5_video[dset].shape[0]):
            img = hdf5_video[dset][idx]
            keypoints = hdf5_tracking[dset+'-keypoints'][idx]
            confidence = hdf5_tracking[dset+'-confidence'][idx]

            # Joints listed here: https://github.com/CMU-Perceptual-Computing-Lab/openpose/
            # blob/master/doc/02_output.md#keypoints-in-cpython
            for joint in (4, 7):
                x = int(keypoints[joint][0])
                y = int(keypoints[joint][1])
                img = cv2.circle(img, (x, y), 20,
                                 colorScale(confidence[4], 0, 1))

            video_writer.write(img)
        video_writer.release()
    hdf5_video.close()
    hdf5_tracking.close()


# Expect one argument of form: <file stub> which will be used to access
# <file stub>.hdf5 and <filestub>-novid.hdf5 and create <file stub>-wrists.avi
if __name__ == '__main__':
    overlay(sys.argv[1])
