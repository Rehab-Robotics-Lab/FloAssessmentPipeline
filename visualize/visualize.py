"""Module to orchestrate visualizations"""
#!/usr/bin/env python3

import argparse
from visualize.video_overlay.overlay_wrists import overlay_wrists
from visualize.video_overlay.skeleton_overlay2d import overlay_2d_skeleton
from visualize.skeleton_3d import skeleton_3d
from visualize.video_overlay.overlay_angular_motion import overlay_angular_motion


def visualize(directory, cam, func, algorithms):
    """Visualize data

    Ingests two hdf5 files and outputs video of the tracking in them.

    The angular_motion function requires that angular motion have been extracted.

    Args:
        directory: directory with both full_data-novid.hdf5 and full_data-vid.hdf5
        cam: The camera to use (upper, lower)
        func: The visualization function to use (wrists, 2dSkeleton, 3dSkeleton,
              angular_motion)
        algorithms: The algorithms to put onto plots
    """
    dset_names = {}
    dset_names['cam_root'] = f'vid/{cam}'
    dset_names['color_dset'] = f'{dset_names["cam_root"]}/color/data'
    dset_names['depth_dset'] = f'{dset_names["cam_root"]}/depth/data'
    dset_names['depth_mapping_dset'] = \
        f'{dset_names["cam_root"]}/color/matched_depth_index'
    dset_names['time_color'] = f'{dset_names["cam_root"]}/color/time'
    dset_names['time_depth'] = f'{dset_names["cam_root"]}/depth/time'
    for alg in algorithms:
        dset_names[alg] = {}
        # not all of these exist for all algorithm outputs. It is up
        # to the user to use the right things
        dset_names[alg]['root'] = f'{dset_names["cam_root"]}/pose/{alg}'
        dset_names[alg]['left'] = {}
        dset_names[alg]['right'] = {}
        dset_names[alg]['left']['root'] = f'{dset_names["cam_root"]}/pose/{alg}/left'
        dset_names[alg]['right']['root'] = f'{dset_names["cam_root"]}/pose/{alg}/right'

        dset_names[alg]['3dkeypoints'] = \
            f'{dset_names[alg]["root"]}/3dkeypoints/raw_realsense'
        dset_names[alg]['keypoints_color'] = \
            f'{dset_names[alg]["root"]}/keypoints/color'
        dset_names[alg]['keypoints_depth'] = \
            f'{dset_names[alg]["root"]}/keypoints/depth'
        dset_names[alg]['confidence'] = \
            f'{dset_names[alg]["root"]}/confidence'

        for arm in ('left', 'right'):
            dset_names[alg][arm]['3dkeypoints'] = \
                f'{dset_names[alg][arm]["root"]}/3dkeypoints/raw_realsense'
            dset_names[alg][arm]['keypoints_color'] = \
                f'{dset_names[alg][arm]["root"]}/keypoints/color'
            dset_names[alg][arm]['keypoints_depth'] = \
                f'{dset_names[alg][arm]["root"]}/keypoints/depth'
            dset_names[alg][arm]['confidence'] = \
                f'{dset_names[alg][arm]["root"]}/confidence'

    if func == 'wrists':
        print('overlaying wrists')
        if len(algorithms) > 1:
            raise ValueError("Wrists overlay can only plot from one algorithm")
        overlay_wrists(directory, cam, dset_names, algorithms[0])

    elif func == '2dSkeleton':
        print('overlaying 2D Skeleton')
        overlay_2d_skeleton(directory, cam, dset_names, algorithms)

    elif func == '3dSkeleton':
        print('creating 3d skeleton GIF')
        skeleton_3d(directory, cam, dset_names,
                    algorithms, save=True, show=False)

    elif func == 'angular_motion':
        print('Plotting Angular motion at Shoulders')
        overlay_angular_motion(directory, cam, dset_names, algorithms)

    else:
        print('Invalid OVERLAY option')


if __name__ == '__main__':
    PARSER = argparse.ArgumentParser()

    PARSER.add_argument('-d', '--dir', type=str, required=True,
                        help='The directory to find the files to process ' +
                        'Two HDF5 files are expected: <dir>/full_data-vid.hdf5 ' +
                        'and <dir>/full_data-novid.hdf5. The result will be ' +
                        'generated in the same directory with filename: ' +
                        '<dir>/viz-<cam>-<function>.avi')
    PARSER.add_argument('-c', '--cam', type=str, choices=['upper', 'lower'],
                        required=True,
                        help='which camera to use')
    PARSER.add_argument('-f', '--function', type=str, required=True,
                        choices=['wrists', '2dSkeleton',
                                 '3dSkeleton', 'angular_motion'],
                        help='which visualization function to use')
    PARSER.add_argument('-a', '--algorithm', type=str,
                        action='append', required=True,
                        choices=['openpose:25B', 'mp-hands'])
    ARGS = PARSER.parse_args()
    visualize(ARGS.dir, ARGS.cam, ARGS.function, ARGS.algorithm)
