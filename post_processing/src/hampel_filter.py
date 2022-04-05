import numpy as np
# outlier rejection: find iqr for 10 points above and below point
# if point outside of median+IQR, then replace with median

# TODO: these produce different results, why?


def hampel_filter(data, half_win_length, threshold):
    data = np.ma.array(data)
    data[np.isnan(data)] = np.ma.masked
    bad_idx = np.array([], dtype='int')
    for (idx, dp), not_valid in zip(enumerate(data), data.mask):
        if np.isnan(dp):
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
    padded_data = np.concatenate(
        [[np.nan]*half_win_length, data, [np.nan]*half_win_length])
    windows = np.ma.array(
        np.lib.stride_tricks.sliding_window_view(padded_data, 2*50+1))
    windows[np.isnan(windows)] = np.ma.masked
    median = np.ma.median(windows, axis=1)
    mad = np.ma.median(np.abs(windows-np.atleast_2d(median).T), axis=1)
    bad = np.abs(data-median) > (mad*threshold)
    return np.where(bad)[0]
