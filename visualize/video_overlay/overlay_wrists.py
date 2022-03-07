"""Module to overlay wrists"""
#!/usr/bin/env python3

import pathlib
import h5py
from tqdm import trange
import cv2
from common import img_overlays
from common import color
from pose.src.openpose_joints import openpose_joints


def overlay_wrists(directory, cam, dset_names):
    """Overlay wrists on the color image

    Requires the two hdf5 files: full_data-novid.hdf5
    and full_data-vid.hdf5

    Args:
        directory: Directory with the hdf5 files
        cam: The camera to use
        dset_names: The names of the relevant datasets
    """
    directory = pathlib.Path(directory)
    hdf5_video = h5py.File(directory/'full_data-vid.hdf5', 'r')
    hdf5_tracking = h5py.File(directory/'full_data-novid.hdf5', 'r')

    video_writer = cv2.VideoWriter(
        str(directory/f'viz-{cam}-wrists.avi'),
        cv2.VideoWriter_fourcc(*'MJPG'), 30, (1920, 1080))
    left_wrist_plots = [img_overlays.DataPlot(
        (10, 30), (400, 200), label="left wrist: x", color=(0, 200, 0), blur=25),
        img_overlays.DataPlot(
        (10, 240), (400, 200), label="left wrist: y", color=(200, 0, 0), blur=25)]
    right_wrist_plots = [img_overlays.DataPlot(
        (10, 450), (400, 200), label="right wrist: x", color=(0, 0, 200), blur=25),
        img_overlays.DataPlot(
        (10, 660), (400, 200), label="right wrist: y", color=(200, 200, 0), blur=25)]
    for idx in trange(hdf5_video[dset_names['color_dset']].shape[0]):
        img = hdf5_video[dset_names['color_dset']][idx]
        keypoints = hdf5_tracking[dset_names['keypoints_color']][idx]
        confidence = hdf5_tracking[dset_names['confidence']][idx]
        time = hdf5_tracking[dset_names['time_color']][idx]

        img_overlays.draw_cam_info(img, idx, time, cam)

        for joint, plts in zip(
                (openpose_joints().index("LWrist"),
                 openpose_joints().index("RWrist")),
                (left_wrist_plots, right_wrist_plots)):
            x_pos = int(keypoints[joint][0])
            y_pos = int(keypoints[joint][1])
            cv2.circle(img, (x_pos, y_pos), 20,
                       color.color_scale(confidence[joint], 0, 1), 8)
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
