import scipy
from post_processing.src import hdf5_tools
from post_processing.src import plotting
from post_processing.src import filter_joints
import matplotlib.pyplot as plt
import numpy as np
import multiprocessing
from platform import python_version
import h5py
import pathlib
import pandas as pd

from post_processing.src import hdf5_tools
from post_processing.src import arm_length


target_dir = pathlib.Path("/media/mjsobrep/43CDA61E672B9161/pose/")

hdf5_file = h5py.File(target_dir/"smoothed_data.hdf5", 'r')
train = pd.read_csv(target_dir/"train.csv")

results = []

threshold = 5
for subj in [f'0{id}' for id in train['record_id']]:
    for game_name in hdf5_file[subj]:
        games = hdf5_file[subj][game_name]
        for rep in games:
            game_rep = games[rep]
            if 'smooth' in game_rep:
                for side in ['R', 'L']:
                    joint = f'{side}Wrist'
                    # we only really want to filter by position variance
                    low_conf_values = arm_length.filter_by_variance(
                        game_rep['covariance'][joint][:, :3, :3])
                    masked_data = np.ma.array(game_rep['smooth'][joint])
                    masked_data[low_conf_values] = np.ma.masked
                    if 'Wrist' in joint:
                        side = joint[0]
                        shoulder_data = game_rep['smooth'][f'{side}Shoulder'][(
                            ~low_conf_values).nonzero()[0]][:, 0:3]
                        wrist_data = game_rep['smooth'][f'{side}Wrist'][(
                            ~low_conf_values).nonzero()[0]][:, 0:3]
                        ch = scipy.spatial.ConvexHull(wrist_data-shoulder_data)
                        arm_l = arm_length.arm_length(game_rep, side)
                        norm_ch = ch.volume/(4/3*np.pi*arm_l**3)
                        print(f'For Subj: {subj}:')
                        subj_norms = train[train["record_id"] == int(subj)]
                        bbt = subj_norms["bbt.z_right"].values[0]
                        age = subj_norms["age"].values[0]
                        print(
                            f'{side} Arm bbt: {bbt}; age: {age}; norm convex hull: {norm_ch}; arm length: {arm_l}')
                        results.append(
                            [subj, side, bbt, age, ch.volume, arm_l, norm_ch])

results_df = pd.DataFrame(results, columns=[
                          'subject', 'side', 'bbt', 'age', 'convex_hull', 'arm_length', 'norm_convex_hull'])
results_df.to_csv(target_dir/'ss_ch.csv')
