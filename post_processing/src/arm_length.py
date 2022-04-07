
import numpy as np


def filter_by_variance(covariances, threshold=5):
    """Find values which have variances higher than the
    median covariance + threshold * median absolution deviation of covariance
    Only consider the diagonals (variance) of the covariance matrix

    Args:
        covariances: covariance matrices ([n-timestamps, n-features*n-dim,n-features*n-dim])
                     this will use the first three dimmensions. Only pass in the dimmensions
                     of the covariance that you want considered (ie [:,:3,:3] for xyz)
        threshold: The threshold to apply on top of the MAD

    Returns: mask of bad values
    """
    variances = np.diagonal(covariances, axis1=1, axis2=2)
    med = np.median(variances, axis=0)
    mad = np.median(np.abs(variances-med), axis=0)
    # only want to remove values with larger covariance, smaller variance is fine
    return np.any((variances-med) > (mad*threshold), axis=1)


def arm_length(data, side):
    # filter to only get the highest confidence points
    threshold = 3

    wrist_bad_vals = filter_by_variance(
        data['covariance'][f'{side}Wrist'][:, :3], threshold=threshold)
    elbow_bad_vals = filter_by_variance(
        data['covariance'][f'{side}Elbow'][:, :3], threshold=threshold)
    shoulder_bad_vals = filter_by_variance(
        data['covariance'][f'{side}Shoulder'][:, :3], threshold=threshold)

    wrist_data = data['smooth'][f'{side}Wrist'][:, :3]
    elbow_data = data['smooth'][f'{side}Elbow'][:, :3]
    shoulder_data = data['smooth'][f'{side}Shoulder'][:, :3]

    bad_vals = np.any([wrist_bad_vals, elbow_bad_vals,
                       shoulder_bad_vals], axis=0)
    wrist_data = wrist_data[~bad_vals]
    elbow_data = elbow_data[~bad_vals]
    shoulder_data = shoulder_data[~bad_vals]

    forearm_length = np.sqrt(np.sum((wrist_data-elbow_data)**2, axis=1))
    upper_arm_length = np.sqrt(np.sum((elbow_data-shoulder_data)**2, axis=1))
    med_arm_length = np.median(forearm_length+upper_arm_length)
    return med_arm_length
