"""Module to overlay wrists"""
#!/usr/bin/env python3

import pathlib
import h5py
from tqdm import trange
import cv2
from common import img_overlays
from pose.src.joints import openpose_joints, mphands_joints

LEFT_HAND_COLOR = (176, 122, 28)
RIGHT_HAND_COLOR = (186, 26, 183)


def overlay_wrists(directory, cam, dset_names, algorithm):
    """Overlay wrists on the color image

    Requires the two hdf5 files: full_data-novid.hdf5
    and full_data-vid.hdf5

    Args:
        directory: Directory with the hdf5 files
        cam: The camera to use
        dset_names: The names of the relevant datasets
        algorithm: Algorithm to plot
    """
    #pylint: disable=too-many-locals
    directory = pathlib.Path(directory)
    hdf5_video = h5py.File(directory/'full_data-vid.hdf5', 'r')
    hdf5_tracking = h5py.File(directory/'full_data-novid.hdf5', 'r')

    video_writer = cv2.VideoWriter(
        str(directory/f'viz-{cam}-{algorithm}-wrists.avi'),
        cv2.VideoWriter_fourcc(*'MJPG'), 30, (1920, 1080))
    left_wrist_plots = [img_overlays.DataPlot(
        (10, 30), (400, 200), label="left wrist: x", color=LEFT_HAND_COLOR, blur=25),
        img_overlays.DataPlot(
        (10, 240), (400, 200), label="left wrist: y", color=LEFT_HAND_COLOR, blur=25)]
    right_wrist_plots = [img_overlays.DataPlot(
        (10, 450), (400, 200), label="right wrist: x", color=RIGHT_HAND_COLOR, blur=25),
        img_overlays.DataPlot(
        (10, 660), (400, 200), label="right wrist: y", color=RIGHT_HAND_COLOR, blur=25)]

    for idx in trange(hdf5_video[dset_names['color_dset']].shape[0]):
        img = hdf5_video[dset_names['color_dset']][idx]
        time = hdf5_tracking[dset_names['time_color']][idx]

        img_overlays.draw_cam_info(img, idx, time, cam)
        img_overlays.draw_text(img, f'algorithm: {algorithm}', pos=(1400, 3))

        if 'openpose:25' in algorithm:
            keypoints = hdf5_tracking[dset_names[algorithm]
                                      ['keypoints_color']][idx]
            left = keypoints[openpose_joints().index("LWrist")]
            right = keypoints[openpose_joints().index("RWrist")]
        elif algorithm == 'mp-hands':
            left = \
                hdf5_tracking[dset_names[algorithm]
                              ["left"]["keypoints_color"]][idx][mphands_joints().index("WRIST")]
            right = \
                hdf5_tracking[dset_names[algorithm]
                              ["right"]["keypoints_color"]][idx][mphands_joints().index("WRIST")]

        for joint, plts, hand_color in zip(
                (left, right),
                (left_wrist_plots, right_wrist_plots),
                (LEFT_HAND_COLOR, RIGHT_HAND_COLOR)):
            x_pos = int(joint[0])
            y_pos = int(joint[1])
            cv2.circle(img, (x_pos, y_pos), 20, hand_color, 8)
            # x
            plts[0].add_data(time, x_pos)
            plts[0].plot(img)
            # y
            plts[1].add_data(time, y_pos)
            plts[1].plot(img)

        video_writer.write(img)
    video_writer.release()
    hdf5_video.close()
    hdf5_tracking.close()
