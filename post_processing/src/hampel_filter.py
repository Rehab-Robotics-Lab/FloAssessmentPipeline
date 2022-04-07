"""Module for running hampel filter"""
import numpy as np

# TODO: these produce different results, why?


def hampel_filter(data, half_win_length, threshold):
    """Apply hampel filter to data to discover outliers in data

    This is a loop based version, runs slow

    For each window, the median and median absolute difference (mad)
    are calculated. If the target value (center of window) is
    outside of the median +/- (mad*threshold) then it is taken
    to be a bad value. No filling is done, bad values are just
    returned.

    Args:
        data: The data to evaluate
        half_win_length: Half of the window length to process
        threshold: The threshold to apply on top of MAD
    Returns: the indices which are no good
    """
    data = np.ma.array(data)
    data[np.isnan(data)] = np.ma.masked
    bad_idx = np.array([], dtype='int')
    for (idx, dat), not_valid in zip(enumerate(data), data.mask):
        if np.isnan(dat):
            continue
        if not_valid:
            continue
        lower_bound = np.max([0, idx-half_win_length])
        upper_bound = np.min([len(data)-1, idx+half_win_length])
        windowed_data = data[lower_bound:upper_bound]
        median = np.ma.median(windowed_data)
        mad = np.ma.median(np.abs(windowed_data-median))
        if np.abs(data[idx]-median) > mad*threshold:
            bad_idx = np.append(bad_idx, idx)
#     return (bad_idx, mad_arr, med_arr)
    return bad_idx


def hampel_filter_v(data, half_win_length, threshold):
    """Apply hampel filter to data to discover outliers in data

    This is a vectorized version. Runs fast.

    For each window, the median and median absolute difference (mad)
    are calculated. If the target value (center of window) is
    outside of the median +/- (mad*threshold) then it is taken
    to be a bad value. No filling is done, bad values are just
    returned.

    Args:
        data: The data to evaluate
        half_win_length: Half of the window length to process
        threshold: The threshold to apply on top of MAD
    Returns: the indices which are no good
    """
    padded_data = np.concatenate(
        [[np.nan]*half_win_length, data, [np.nan]*half_win_length])
    windows = np.ma.array(
        np.lib.stride_tricks.sliding_window_view(padded_data, 2*50+1))
    windows[np.isnan(windows)] = np.ma.masked
    median = np.ma.median(windows, axis=1)
    mad = np.ma.median(np.abs(windows-np.atleast_2d(median).T), axis=1)
    bad = np.abs(data-median) > (mad*threshold)
    return np.where(bad)[0]
