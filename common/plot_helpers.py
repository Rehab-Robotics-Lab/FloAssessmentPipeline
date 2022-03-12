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
        points3d: Array of 3D points shape: (n, 3 (x,y,z))

    Return: (max_range, x_percentiles, y_percentiles, z_percentiles)
    """
    x_percentiles = np.nanpercentile(points3d[:, 0], [1, 50, 99])
    x_diff = x_percentiles[-1]-x_percentiles[0]
    y_percentiles = np.nanpercentile(points3d[:, 1], [1, 50, 99])
    y_diff = y_percentiles[-1]-y_percentiles[0]
    z_percentiles = np.nanpercentile(points3d[:, 2], [1, 50, 99])
    z_diff = z_percentiles[-1]-z_percentiles[0]
    max_range = np.max((x_diff, y_diff, z_diff))
    return max_range, x_percentiles, y_percentiles, z_percentiles


def stretch_histogram(img):
    """Stretch histogram of 16 bit single channel image

    https://docs.opencv.org/4.x/d5/daf/tutorial_py_histogram_equalization.html
    this does the same thing as cv2.equalizeHist, but for 16 bit images
    (as opposed to 8 bit)

    Args:
        img: The image to stretch

    Returns: the image, stretched
    """
    hist, _ = np.histogram(img.flatten(), 0xFFFF+1, [0, 0xFFFF+1])
    cdf = hist.cumsum()
    # cdf_normalized = cdf * float(hist.max()) / cdf.max()
    cdf_m = np.ma.masked_equal(cdf, 0)
    cdf_m = (cdf_m - cdf_m.min())*0xFFFF/(cdf_m.max()-cdf_m.min())
    cdf = np.ma.filled(cdf_m, 0).astype('uint16')
    return cdf[img]
