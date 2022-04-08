"""Module for generating test-train split

This replicates what is done in notebooks/test_train_split
"""
import argparse
import pathlib
import pandas as pd
import numpy as np
import h5py


def rand_select(data, rng):
    """Given a pandas data frame, split it in half by the median
    box and block score. From each half, select one subject to
    be part of a training set and the rest to be for a test set.
    Return both sets

    Args:
        data: Pandas data frame with `min_bbt` field at least
        rng: np.random.RandomState object
    Returns: (train, test)
    """
    med = data['min_bbt'].median()
    lower_group = data[data['min_bbt'] <= med]
    lower_test = lower_group.sample(1, random_state=rng)
    lower_train = lower_group.drop(lower_test.index)
    upper_group = data[data['min_bbt'] > med]
    upper_test = upper_group.sample(1, random_state=rng)
    upper_train = upper_group.drop(upper_test.index)
    test = pd.concat([lower_test, upper_test])
    train = pd.concat([lower_train, upper_train])
    return train, test


def test_train_split(target_dir, random_seed=24402):
    """Generate test train sets for subjects. Will load subjects from
    the `data_labels.csv` file and make sure that the subject has
    corresponding smoothed and filtered pose data. If they do not,
    they are excluded. Then, for children, young adults, adults, and elders
    one subject for each group above the median minumum bbt z-score and
    one below the median are randomly selected to be part of the test
    set. The remainder of the subjects are assigned to the training
    set.


    Args:
        target_dir: The directory to find the data_labels.csv and
                    smoothed_data.hdf5
        random_seed: The random seed to use when randomly making
                     selections.
    """
    target_dir = pathlib.Path(target_dir)
    data_labels = pd.read_csv(target_dir/"data_labels.csv")
    hdf5_file = h5py.File(target_dir/"smoothed_data.hdf5", 'r')

    data_labels = data_labels[data_labels['record_id'].isin(
        [int(id) for id in hdf5_file if id[0] == '0'])]

    data_labels['min_bbt'] = data_labels.filter(
        items=['bbt.z_left', 'bbt.z_right']).min(axis=1)

    # For each age group, select 1 subject randomly below median and
    # one above. Those are the test set

    children = data_labels[data_labels['age'] < 12]
    young_adult = data_labels[(data_labels['age'] >= 12)
                              & (data_labels['age'] < 21)]
    adult = data_labels[(data_labels['age'] >= 21) & (data_labels['age'] < 65)]
    elder = data_labels[(data_labels['age'] >= 65)]

    rng = np.random.RandomState(random_seed)
    children_train, children_test = rand_select(children, rng)
    ya_train, ya_test = rand_select(young_adult, rng)
    adult_train, adult_test = rand_select(adult, rng)
    elder_train, elder_test = rand_select(elder, rng)

    train = pd.concat([children_train, ya_train, adult_train, elder_train])
    test = pd.concat([children_test, ya_test, adult_test, elder_test])

    test.to_csv(target_dir/'test.csv')
    train.to_csv(target_dir/'train.csv')


if __name__ == '__main__':
    TT_PARSER = argparse.ArgumentParser()

    TT_PARSER.add_argument("-t", "--target", type=str, required=True,
                           help="where to find the hdf5 files and save the result")
    ARGS = TT_PARSER.parse_args()
    test_train_split(ARGS.target)
