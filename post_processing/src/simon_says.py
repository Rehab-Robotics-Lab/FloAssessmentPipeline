"""Module for calculating convex hull of arm motion from simon says game"""

import argparse
import pathlib
import h5py
import numpy as np
import scipy.spatial
import pandas as pd
from post_processing.src import arm_length


def simon_says_convex_hell(target_dir, dset, threshold=5):
    """Calculate the convex hull for arm movement for simon says games

    1. Loads data from target_dir/smoothed_data.hdf5 (generated in generate_state.py)
    2. Determines which values to trust by taking values which are less than
       threshold*MAD+median for variances for the wrist.
    3. Determines arm length, taking the median of sum of segment lengths
    4. Calculates the convex hull of the arms as the wrist positions-shoulder positions
    5. Normalizes by dividing by maximum possible volume
    6. Saves results to target_dir/ss_ch.csv

    Args:
        target_dir: The directory to process in
        dset: dataset to work on (`test` or `train`)
        threshold: The threshold for which values to trust
    """
    #pylint: disable=too-many-locals
    target_dir = pathlib.Path(target_dir)

    hdf5_file = h5py.File(target_dir/"smoothed_data.hdf5", 'r')
    data_set = pd.read_csv(target_dir/f"{dset}.csv")

    results = []

    for subj in [f'{id:03}' for id in data_set['record_id']]:
        for game_name in hdf5_file[subj]:
            if not game_name == 'simon_says':
                continue
            games = hdf5_file[subj][game_name]
            for rep in games:
                game_rep = games[rep]
                if 'smooth' in game_rep:
                    for side in ['R', 'L']:
                        joint = f'{side}Wrist'
                        # we only really want to filter by position variance
                        low_conf_values = arm_length.filter_by_variance(
                            game_rep['covariance'][joint][:, :3, :3],
                            threshold=threshold)
                        masked_data = np.ma.array(game_rep['smooth'][joint])
                        masked_data[low_conf_values] = np.ma.masked
                        shoulder_data = game_rep['smooth'][f'{side}Shoulder'][(
                            ~low_conf_values).nonzero()[0]][:, 0:3]
                        wrist_data = game_rep['smooth'][f'{side}Wrist'][(
                            ~low_conf_values).nonzero()[0]][:, 0:3]
                        conv_hull = scipy.spatial.ConvexHull(
                            wrist_data-shoulder_data)
                        arm_l = arm_length.arm_length(game_rep, side)
                        norm_ch_vol = conv_hull.volume/(4/3*np.pi*arm_l**3)
                        norm_ch_sa = conv_hull.area/(4*np.pi*arm_l**2)
                        subj_norms = data_set[data_set["record_id"] == int(
                            subj)]
                        bbt = subj_norms[f"bbt.z_{'right' if side=='R' else 'left'}"].values[0]
                        age = subj_norms["age"].values[0]
                        print(
                            f'subj: {subj}; {side} Arm bbt: {bbt}; age: {age}; ' +
                            f'norm convex hull: {norm_ch_vol}; arm length: {arm_l}')
                        results.append(
                            [subj, rep, side, bbt, age, conv_hull.volume,
                                arm_l, norm_ch_vol, conv_hull.area, norm_ch_sa])

    results_df = pd.DataFrame(results, columns=[
                              'subject', 'rep', 'side', 'bbt', 'age', 'convex_hull',
                              'arm_length', 'norm_convex_hull',
                              'convex_hull_surface_area', 'convex_hull_surface_area_norm'])
    results_df.to_csv(target_dir/f'ss_ch-{dset}.csv')

    # target_dir = pathlib.Path("/media/mjsobrep/43CDA61E672B9161/pose/")
if __name__ == '__main__':
    SS_PARSER = argparse.ArgumentParser()

    SS_PARSER.add_argument("-t", "--target", type=str, required=True,
                           help="where to find the hdf5 files and save the result")
    SS_PARSER.add_argument('-d', '--dataset', type=str, required=True,
                           choices=['test', 'train'],
                           help='Whether to run on the test or train dataset')
    ARGS = SS_PARSER.parse_args()
    simon_says_convex_hell(ARGS.target, ARGS.dataset)
