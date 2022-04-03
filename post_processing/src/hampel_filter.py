import numpy as np
# outlier rejection: find iqr for 10 points above and below point
# if point outside of median+IQR, then replace with median


def hampel_filter(data, half_win_length, threshold):
    data = np.ma.array(data)
    data[np.isnan(data)] = np.ma.masked
    bad_idx = np.array([], dtype='int')
    mad_arr = np.zeros((len(data)))
    med_arr = np.zeros((len(data)))
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
        mad_arr[idx] = mad
        med_arr[idx] = np.abs(data[idx]-median)
        if np.abs(data[idx]-median) > mad*threshold:
            bad_idx = np.append(bad_idx, idx)
#     return (bad_idx, mad_arr, med_arr)
    return bad_idx
