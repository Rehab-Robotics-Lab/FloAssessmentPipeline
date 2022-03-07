"""Functions to make plotting easier"""

import numpy as np


def calculate_data_range(points3d):
    """Calculate the range of data in 3D space
    over a timeseries

    calculates percentiles in the x, y, z direction at
    1%, 50%, 99%. These are useful for centering your data.

    calculates the max absolute range by looking in each direction
    at the distance from the max and min value and taking the max
    over all 3 axes.

    Args:
        points3d: Array of 3D points

    Return: (max_range, x_percentiles, y_percentiles, z_percentiles)
    """
    x_percentiles = np.nanpercentile(points3d[:, :, 0], [1, 50, 99])
    x_diff = x_percentiles[-1]-x_percentiles[0]
    y_percentiles = np.nanpercentile(points3d[:, :, 1], [1, 50, 99])
    y_diff = y_percentiles[-1]-y_percentiles[0]
    z_percentiles = np.nanpercentile(points3d[:, :, 2], [1, 50, 99])
    z_diff = z_percentiles[-1]-z_percentiles[0]
    max_range = np.max((x_diff, y_diff, z_diff))
    return max_range, x_percentiles, y_percentiles, z_percentiles
