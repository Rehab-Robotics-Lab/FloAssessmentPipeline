"""Module for calculating convex hull of arm motion from target touch game"""

import argparse
import pathlib
import h5py
import numpy as np
import scipy.signal
import scipy.spatial
import pandas as pd
from post_processing.src import arm_length


def target_touch_accel(target_dir):
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
        threshold: The threshold for which values to trust
    """
    #pylint: disable=too-many-locals
    target_dir = pathlib.Path(target_dir)

    hdf5_file = h5py.File(target_dir/"smoothed_data.hdf5", 'r')
    train = pd.read_csv(target_dir/"train.csv")

    results = []

    for subj in [f'{id:03}' for id in train['record_id']]:
        for game_name in hdf5_file[subj]:
            if not game_name == 'target_touch':
                continue
            games = hdf5_file[subj][game_name]
            for rep in games:
                game_rep = games[rep]
                if 'smooth' in game_rep:
                    for joint in ['right', 'left']:
                        # we only really want to filter by position variance
                        data = game_rep['smooth'][joint]
                        time = game_rep['time']

                        velocity = np.sqrt(
                            np.sum(np.power(data[:, 3:6], 2), axis=1))
                        acceleration = np.sqrt(
                            np.sum(np.power(data[:, 6:9], 2), axis=1))

                        edges = np.diff(1*(velocity > 100))
                        start_idx = np.where(edges == 1)[0]
                        end_idx = np.where(edges == -1)[0]
                        if len(start_idx) == 0 or len(end_idx) == 0:
                            continue
                        if start_idx[0] > end_idx[0]:
                            start_idx = np.insert(start_idx, 0, 0)
                        if len(start_idx) > len(end_idx):
                            end_idx = np.append(end_idx, len(data)-1)

                        start_idx_2 = []
                        end_idx_2 = []
                        for start, end in zip(start_idx, end_idx):
                            local_minima = start + \
                                scipy.signal.argrelextrema(
                                    velocity[start:end], np.less)[0]
                            local_minima = np.insert(local_minima, 0, start)
                            local_minima = np.append(local_minima, end)
                            for idx in range(len(local_minima)-1):
                                start_idx_2.append(local_minima[idx])
                                end_idx_2.append(local_minima[idx+1])
                        start_idx = np.array(start_idx_2)
                        end_idx = np.array(end_idx_2)

                        real_movements = [
                            np.trapz(velocity[start:end],
                                     x=time[start:end]) > 200
                            for start, end in zip(start_idx, end_idx)]
                        start_idx = start_idx[real_movements]
                        end_idx = end_idx[real_movements]

                        max_accel = []
                        max_div_avg_accel = []
                        time_to_max_accel = []
                        time_to_max_accel_norm = []
                        max_vel = []
                        time_to_max_vel = []
                        time_to_max_vel_norm = []
                        max_div_avg_vel = []
                        length_movement_xyz = []
                        length_movement_t = []
                        num_movements = len(start_idx)

                        completed_segment = False
                        for start, end in zip(start_idx, end_idx):
                            seg_accel = acceleration[start:end]
                            seg_vel = velocity[start:end]
                            seg_time = time[start:end]
                            seg_length_time = time[end]-time[start]
                            if seg_length_time < 0.1:
                                continue
                            area_of_accel = np.trapz(seg_accel, x=seg_time)
                            avg_accel = area_of_accel/seg_length_time
                            max_a = np.max(seg_accel)
                            max_accel.append(max_a)
                            max_div_avg_accel.append(max_a/avg_accel)
                            ttma = time[np.argmax(seg_accel)+start]-time[start]
                            time_to_max_accel.append(ttma)
                            time_to_max_accel_norm.append(ttma/seg_length_time)
                            max_v = np.max(seg_vel)
                            max_vel.append(max_v)
                            ttmv = time[np.argmax(seg_vel)+start]-time[start]
                            time_to_max_vel.append(ttmv)
                            time_to_max_vel_norm.append(ttmv/(seg_length_time))
                            area_of_vel = np.trapz(seg_vel, x=seg_time)
                            avg_vel = area_of_vel/seg_length_time
                            max_div_avg_vel.append(max_v/avg_vel)
                            length_movement_xyz.append(area_of_vel)
                            length_movement_t.append(seg_length_time)
                            completed_segment = True

                            # ttmv = time[np.argmax(velocity)]
                        if not completed_segment:
                            continue

                        # max_div_avg_accel = np.ma.array(max_div_avg_accel)
                        # max_div_avg_accel[np.isinf(
                        #     max_div_avg_accel)] = np.ma.masked
                        # med_max_avg_accel = np.ma.median(max_div_avg_accel)
                        subj_norms = train[train["record_id"] == int(subj)]
                        bbt = subj_norms[f"bbt.z_{joint}"].values[0]
                        age = subj_norms["age"].values[0]

                        # print(
                        #     f'subj: {subj}; {joint} Arm bbt: {bbt}; age: {age}; ' +
                        #     f'max/avg accel: {med_max_avg_accel}')
                        results.append(
                            [subj,
                                rep,
                                joint,
                                bbt,
                                age,
                                np.median(max_accel),
                                np.median(max_div_avg_accel),
                                np.median(time_to_max_accel),
                                np.median(time_to_max_accel_norm),
                                np.median(max_vel),
                                np.median(time_to_max_vel),
                                np.median(time_to_max_vel_norm),
                                np.median(max_div_avg_vel),
                                np.median(length_movement_xyz),
                                np.median(length_movement_t),
                                num_movements])

    results_df = pd.DataFrame(results, columns=[
                              'subject',
                              'rep',
                              'side',
                              'bbt',
                              'age',
                              'max_acceleration',
                              'max_acceleration_over_avg_acceleration',
                              'time_to_max_acceleration',
                              'time_to_max_acceleration_normalized',
                              'max_velocity',
                              'time_to_max_velocity',
                              'time_to_max_velocity_normalized',
                              'max_velocity_over_avg_velocity',
                              'distance_traveled',
                              'movement_time_length',
                              'number_movements'])
    results_df.to_csv(target_dir/'tt_features.csv')

    # target_dir = pathlib.Path("/media/mjsobrep/43CDA61E672B9161/pose/")
if __name__ == '__main__':
    TT_PARSER = argparse.ArgumentParser()

    TT_PARSER.add_argument("-t", "--target", type=str, required=True,
                           help="where to find the hdf5 files and save the result")
    ARGS = TT_PARSER.parse_args()
    target_touch_accel(ARGS.target)
