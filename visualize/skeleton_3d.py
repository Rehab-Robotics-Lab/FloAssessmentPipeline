"""Module to export pose data as a 3d skeleton"""
#!/usr/bin/env python3

import pathlib
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation
import h5py
import kinematics.scripts.extract_profiles as kinematics
from pose.src.joints import openpose_joints
from common import plot_helpers
# Inspiration from:
# https://medium.com/@pnpsegonne/animating-a-3d-scatterplot-with-matplotlib-ca4b676d4b55

JNTS = openpose_joints()


#pylint: disable=too-many-locals
#pylint: disable=too-many-arguments
def animate(iteration, data, scatters, lines, axes_list, joint_pairs, keypoints, frames):
    """Animate the 3d Skeleton(s)

    Args:
        iteration: the iteration.. duh
        data: the 3d keypoints array to work with. Rows are time instances, columns are keypoints
              and each keypoint has a x,y,z
        scatters: The scatter object with joints
        lines: The lines object connecting joints
        axes_list: The list of axes (for doing subplots)
        joint_pairs: The pairings of joints (limbs)
        keypoints: The keypoints to use (joints)
        frames: The frames for the shoulders
    """
    # efforts to use tqdm to draw a status bar were unsucsefful.
    if iteration % 100 == 0:
        print(f'iteration: {iteration}\t/\t{len(data)}')
    for idx, axes in enumerate(axes_list):
        for jdx, key in enumerate(keypoints):
            # pylint: disable=protected-access
            scatters[idx][jdx]._offsets3d = (
                data[iteration, key, 0:1], data[iteration, key, 1:2], data[iteration, key, 2:])

        for i, line in enumerate(lines[idx]):
            line.set_data([data[iteration, joint_pairs[i][0], 0],
                           data[iteration, joint_pairs[i][1], 0]],
                          [data[iteration, joint_pairs[i][0], 1],
                              data[iteration, joint_pairs[i][1], 1]])
            line.set_3d_properties(
                [data[iteration, joint_pairs[i][0], 2], data[iteration, joint_pairs[i][1], 2]])

        all_quivers = []
        for frame, quivers in frames[idx].items():
            if frame == "right_shoulder_fixed":
                axis_color = ['r', 'g', 'b']
                origin = data[iteration, JNTS.index('RShoulder'), :]
                dirs = kinematics.right_shoulder_fixed_frame(
                    data[iteration, :, :])
            elif frame == "right_shoulder_moving":
                axis_color = ['c', 'm', 'y']
                origin = data[iteration, JNTS.index('RShoulder'), :]
                dirs = kinematics.right_shoulder_moving_frame(
                    data[iteration, :, :])
            elif frame == "left_shoulder_fixed":
                axis_color = ['r', 'g', 'b']
                origin = data[iteration, JNTS.index('LShoulder'), :]
                dirs = kinematics.left_shoulder_fixed_frame(
                    data[iteration, :, :])
            elif frame == "left_shoulder_moving":
                axis_color = ['c', 'm', 'y']
                origin = data[iteration, JNTS.index('LShoulder'), :]
                dirs = kinematics.left_shoulder_moving_frame(
                    data[iteration, :, :])

            new_quivers = []
            for i, quiver in enumerate(quivers):
                quiver.remove()
                quiver = axes.quiver(
                    origin[0], origin[1], origin[2],
                    dirs[0, i], dirs[1, i], dirs[2, i],
                    length=0.1, normalize=True, color=axis_color[i])

                new_quivers.append(quiver)
                all_quivers.append(quiver)

            frames[idx][frame] = new_quivers

    # supposed to return an iterable of all modified artists to enable incremental update
    return scatters, lines, frames, all_quivers


#pylint: disable=too-many-statements
def skeleton_3d(directory, cam, dset_names, algorithms, save=False, show=False):
    """Make animation of 3D skeletons

    Args:
        directory: The directroy to find data ('full_data-novid.hdf5') and
                   to save the output.
        cam: The camera to work with.
        save: Whether or not to save the output
        show: Whether or not to show the output
        algorithms: algorithms to plot
    """
    if len(algorithms) > 1 or ('openpose:25' not in algorithms[0]):
        raise NotImplementedError(
            "Skeleton 3d currently only supports 25 keypoint openpose")

    print('Generating 3D Skeleton')
    directory = pathlib.Path(directory)
    hdf5_tracking = h5py.File(directory/'full_data-novid.hdf5', 'r')

    points3d = hdf5_tracking[dset_names['3dkeypoints']]

    annotations = ['Nose', 'UpperNeck', 'LShoulder', 'LElbow',
                   'LWrist', 'RShoulder', 'RElbow', 'RWrist', 'LHip', 'RHip']
    keypoints = [JNTS.index(ann) for ann in annotations]
    joint_pairs = [
        (JNTS.index('Nose'), JNTS.index('UpperNeck')),
        (JNTS.index('UpperNeck'), JNTS.index('RShoulder')),
        (JNTS.index('UpperNeck'), JNTS.index('LShoulder')),
        (JNTS.index('LShoulder'), JNTS.index('LElbow')),
        (JNTS.index('RShoulder'), JNTS.index('RElbow')),
        (JNTS.index('LElbow'), JNTS.index('LWrist')),
        (JNTS.index('RElbow'), JNTS.index('RWrist')),
        (JNTS.index('UpperNeck'), JNTS.index('LHip')),
        (JNTS.index('UpperNeck'), JNTS.index('RHip'))
    ]

    max_range, x_percentiles, y_percentiles, z_percentiles = \
        plot_helpers.calculate_data_range(points3d)
    plot_radius = 1.2*0.5*max_range

    print('creating figure containers')
    fig = plt.figure(figsize=(20, 10))
    ax1 = fig.add_subplot(2, 2, 1, projection='3d')
    ax2 = fig.add_subplot(2, 2, 2, projection='3d')
    ax3 = fig.add_subplot(2, 2, 3, projection='3d')
    ax4 = fig.add_subplot(2, 2, 4, projection='3d')
    axis_list = (ax1, ax2, ax3, ax4)
    for axis in axis_list:
        axis.margins(x=0)
        axis.margins(y=0)
    print('Done creating subplots')

    views = (
        (0, 0, 'y'),
        (0, 90, 'y'),
        (0, -90, 'z'),
        (15, -125, 'y'),
    )

    scatters = [None for _ in range(len(axis_list))]
    lines = [None for _ in range(len(axis_list))]
    frames = [None for _ in range(len(axis_list))]
    print('creating first frame')
    for idx, axes in enumerate(axis_list):
        # axes = p3.Axes3D(fig)

        # set up the points for the first frame
        scatters[idx] = [
            axes.scatter(points3d[0, i, 0:1],
                         points3d[0, i, 1:2],
                         points3d[0, i, 2:])
            for i in keypoints]

        lines[idx] = [axes.plot([points3d[0, joint[0], 0], points3d[0, joint[1], 0]],
                                [points3d[0, joint[0], 1],
                                    points3d[0, joint[1], 1]],
                                [points3d[0, joint[0], 2],
                                    points3d[0, joint[1], 2]],
                                'black')[0]
                      for joint in joint_pairs]

        # Making frames using quivers to attach to shoulders
        frames[idx] = {}
        axis_color = ['r', 'g', 'b']

        right_shoulder_fixed = kinematics.right_shoulder_fixed_frame(
            points3d[0])
        right_shoulder_fixed_quivers = [axes.quiver(points3d[0, 2, 0],
                                                    points3d[0, 2, 1],
                                                    points3d[0, 2, 2],
                                                    right_shoulder_fixed[0, i],
                                                    right_shoulder_fixed[1, i],
                                                    right_shoulder_fixed[2, i],
                                                    length=0.1, normalize=True, color=axis_color[i])
                                        for i in range(3)]

        right_shoulder = kinematics.right_shoulder_moving_frame(points3d[0])
        right_shoulder_quivers = [axes.quiver(points3d[0, 2, 0],
                                              points3d[0, 2, 1],
                                              points3d[0, 2, 2],
                                              right_shoulder[0, i],
                                              right_shoulder[1, i],
                                              right_shoulder[2, i],
                                              length=0.1, normalize=True, color=axis_color[i])
                                  for i in range(3)]

        left_shoulder_fixed = kinematics.left_shoulder_fixed_frame(points3d[0])
        left_shoulder_fixed_quivers = [axes.quiver(points3d[0, 5, 0],
                                                   points3d[0, 5, 1],
                                                   points3d[0, 5, 2],
                                                   left_shoulder_fixed[0, i],
                                                   left_shoulder_fixed[1, i],
                                                   left_shoulder_fixed[2, i],
                                                   length=0.1, normalize=True, color=axis_color[i])
                                       for i in range(3)]

        left_shoulder = kinematics.left_shoulder_moving_frame(points3d[0])
        left_shoulder_quivers = [axes.quiver(points3d[0, 5, 0],
                                             points3d[0, 5, 1],
                                             points3d[0, 5, 2],
                                             left_shoulder[0, i],
                                             left_shoulder[1, i],
                                             left_shoulder[2, i],
                                             length=0.1, normalize=True, color=axis_color[i])
                                 for i in range(3)]

        frames[idx]["right_shoulder_fixed"] = right_shoulder_fixed_quivers
        frames[idx]["right_shoulder_moving"] = right_shoulder_quivers
        frames[idx]["left_shoulder_fixed"] = left_shoulder_fixed_quivers
        frames[idx]["left_shoulder_moving"] = left_shoulder_quivers

        axes.set_xlim3d(x_percentiles[1]-plot_radius,
                        x_percentiles[1]+plot_radius)
        axes.set_xlabel('X')

        axes.set_ylim3d(y_percentiles[1]+plot_radius,
                        y_percentiles[1]-plot_radius)
        axes.set_ylabel('Y')

        axes.set_zlim3d(z_percentiles[1]-plot_radius,
                        z_percentiles[1]+plot_radius)
        axes.set_zlabel('Z')

        axes.view_init(views[idx][0], views[idx][1],
                       vertical_axis=views[idx][2])

    fig.suptitle('3D Skeleton')
    # ax1.title.set_text('Front')
    # ax2.title.set_text('Left')
    # ax3.title.set_text('Top')
    # ax4.title.set_text('Iso')
    fig.subplots_adjust(left=0, right=1, bottom=0,
                        top=1, wspace=-.1, hspace=-.1)
    ani = animation.FuncAnimation(fig,
                                  animate,
                                  len(points3d),
                                  fargs=(points3d, scatters, lines,
                                         axis_list, joint_pairs, keypoints, frames),
                                  interval=1, blit=False)

    if save:
        writervideo = animation.FFMpegWriter(
            fps=np.floor(
                1/(np.median(np.diff(hdf5_tracking[dset_names['time_color']]))))
        )
        ani.save(directory / f'viz-{cam}-3dSkeleton.avi', writer=writervideo)

    if show:
        plt.show()

    hdf5_tracking.close()
