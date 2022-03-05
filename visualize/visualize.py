#!/usr/bin/env python3
# Expect one argument of form: <file stub> which will be used to access
# <file stub>.hdf5 and <filestub>-novid.hdf5 and create <file stub>-wrists.avi

import argparse
from visualize.video_overlay.overlay_wrists import overlay_wrists
from visualize.video_overlay.skeleton_overlay2d import overlay_2dSkeleton
from visualize.skeleton_3d import skeleton_3d
from visualize.video_overlay.overlay_angular_motion import overlay_angular_motion


def visualize(file_stub, cam, func):
    if func == 'wrists':
        print('overlaying wrists')
        overlay_wrists(file_stub, cam)

    elif func == '2dSkeleton':
        print('overlaying 2D Skeleton')
        overlay_2dSkeleton(file_stub, cam)

    elif func == '3dSkeleton':
        print('creating 3d skeleton GIF')
        skeleton_3d(file_stub, cam, save=True, show=False)

    elif func == 'angular_motion':
        print('Plotting Angular motion at Shoulders')
        overlay_angular_motion(file_stub, cam)

    else:
        print('Invalid OVERLAY option')


if __name__ == '__main__':
    PARSER = argparse.ArgumentParser()

    PARSER.add_argument('--dir', type=str, required=True,
                        help='The directory to find the files to process ' +
                        'Two HDF5 files are expected: <dir>/full_data-vid.hdf5 ' +
                        'and <dir>/full_data-novid.hdf5. The result will be ' +
                        'generated in the same directory with filename: ' +
                        '<dir>/viz-<cam>-<function>.avi')
    PARSER.add_argument('--cam', type=str, choices=['upper', 'lower'],
                        required=True,
                        help='which camera to use')
    PARSER.add_argument('--function', type=str, required=True,
                        choices=['wrists', '2dSkeleton',
                                 '3dSkeleton', 'angular_motion'],
                        help='which visualization function to use')
    ARGS = PARSER.parse_args()
    visualize(ARGS.dir, ARGS.cam, ARGS.function)
