"""Module to orchestrate visualizations"""
#!/usr/bin/env python3

import argparse
from visualize.video_overlay.overlay_wrists import overlay_wrists
from visualize.video_overlay.skeleton_overlay2d import overlay_2d_skeleton
from visualize.skeleton_3d import skeleton_3d
from visualize.video_overlay.overlay_angular_motion import overlay_angular_motion


def visualize(directory, cam, func):
    """Visualize data

    Ingests two hdf5 files and outputs video of the tracking in them.

    The angular_motion function requires that angular motion have been extracted.

    Args:
        directory: directory with both full_data-novid.hdf5 and full_data-vid.hdf5
        cam: The camera to use (upper, lower)
        func: The visualization function to use (wrists, 2dSkeleton, 3dSkeleton,
              angular_motion)
    """
    dset_names = {}
    dset_names['cam_root'] = f'vid/{cam}'
    dset_names['color_dset'] = f'{dset_names["cam_root"]}/color/data'
    dset_names['depth_dset'] = f'{dset_names["cam_root"]}/depth/data'
    dset_names['depth_mapping_dset'] = \
        f'{dset_names["cam_root"]}/color/matched_depth_index'
    dset_names['3dkeypoints'] = \
        f'{dset_names["cam_root"]}/pose/openpose:25B/3dkeypoints/raw_realsense'
    dset_names['keypoints_color'] = \
        f'{dset_names["cam_root"]}/pose/openpose:25B/keypoints/color'
    dset_names['keypoints_depth'] = \
        f'{dset_names["cam_root"]}/pose/openpose:25B/keypoints/depth'
    dset_names['confidence'] = \
        f'{dset_names["cam_root"]}/pose/openpose:25B/confidence'
    dset_names['time_color'] = f'{dset_names["cam_root"]}/color/time'
    dset_names['time_depth'] = f'{dset_names["cam_root"]}/depth/time'

    if func == 'wrists':
        print('overlaying wrists')
        overlay_wrists(directory, cam, dset_names)

    elif func == '2dSkeleton':
        print('overlaying 2D Skeleton')
        overlay_2d_skeleton(directory, cam, dset_names)

    elif func == '3dSkeleton':
        print('creating 3d skeleton GIF')
        skeleton_3d(directory, cam, dset_names, save=True, show=False)

    elif func == 'angular_motion':
        print('Plotting Angular motion at Shoulders')
        overlay_angular_motion(directory, cam, dset_names)

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
