#!/usr/bin/env python3
# Expect one argument of form: <file stub> which will be used to access
# <file stub>.hdf5 and <filestub>-novid.hdf5 and create <file stub>-wrists.avi

import argparse
from overlay_wrists import overlay_wrists
from skeleton_overlay2d import overlay_2dSkeleton
from skeleton_3d import skeleton_3d
from overlay_angular_motion import overlay_angular_motion


def visualize(file_stub, cam, func):
    if func == 'wrists':
        print('overlaying wrists')
        overlay_wrists(file_stub, cam)

    elif func == '2dSkeleton':
        print('overlaying 2D Skeleton')
        overlay_2dSkeleton(file_stub, cam)

    elif func == '3dSkeleton':
        print('creating 3d skeleton GIF')
        skeleton_3d(file_stub, cam, save=False, show=True)

    elif func == 'angular_motion':
        print('Plotting Angular motion at Shoulders')
        overlay_angular_motion(file_stub, cam)

    else:
        print('Invalid OVERLAY option')


if __name__ == '__main__':
    PARSER = argparse.ArgumentParser()

    PARSER.add_argument('--file_stub', type=str,
                        help='root for both the <file_stub>-vid.hdf5 and ' +
                        '<file_stub>-novid.hdf5' +
                        ' files. Will generate <file_stub>-viz.avi')
    PARSER.add_argument('--cam', type=str, choices=['upper', 'lower'],
                        help='which camera to use')
    PARSER.add_argument('--function', type=str,
                        choices=['wrists', '2dSkeleton',
                                 '3dSkeleton', 'angular_motion'],
                        help='which visualization function to use')
    ARGS = PARSER.parse_args()
    visualize(ARGS.file_stub, ARGS.cam, ARGS.function)
