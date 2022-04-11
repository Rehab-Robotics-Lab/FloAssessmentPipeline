"""Module for calculating kinematics of arm motion from target touch game"""

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

    Args:
        target_dir: The directory to process in
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

                        speed = np.sqrt(
                            np.sum(np.power(data[:, 3:6], 2), axis=1))
                        acceleration = np.sqrt(
                            np.sum(np.power(data[:, 6:9], 2), axis=1))
                        # has to be done on raw before taking abs
                        jerk_raw = np.gradient(data[:, 6:9], time, axis=0)
                        jerk = np.sqrt(np.sum(np.power(jerk_raw, 2), axis=1))
                        edges = np.diff(1*(speed > 100))
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
                                    speed[start:end], np.less)[0]
                            local_minima = np.insert(local_minima, 0, start)
                            local_minima = np.append(local_minima, end)
                            for idx in range(len(local_minima)-1):
                                start_idx_2.append(local_minima[idx])
                                end_idx_2.append(local_minima[idx+1])
                        start_idx = np.array(start_idx_2)
                        end_idx = np.array(end_idx_2)

                        real_movements = [
                            np.trapz(speed[start:end],
                                     x=time[start:end]) > 200
                            for start, end in zip(start_idx, end_idx)]
                        start_idx = start_idx[real_movements]
                        end_idx = end_idx[real_movements]

                        # note: mean acceleration is change in velocity. We already know that is small
                        # note: mean velocity is displacement/time
                        max_speed = []
                        max_accel = []
                        max_jerk = []

                        avg_speed = []
                        avg_accel = []
                        avg_jerk = []

                        max_div_avg_speed = []
                        max_div_avg_accel = []
                        max_div_avg_jerk = []

                        time_to_max_speed = []
                        time_to_max_speed_norm = []
                        time_to_max_accel = []
                        time_to_max_accel_norm = []
                        time_to_max_jerk = []
                        time_to_max_jerk_norm = []

                        length_movement_xyz = []  # numerical integral of speed to get path length
                        length_movement_t = []

                        normalized_jerk = []
                        jerk_metric = []
                        speed_metric = []  # mean speed / peek speed

                        num_movements = len(start_idx)

                        completed_segment = False
                        for start, end in zip(start_idx, end_idx):
                            seg_accel = acceleration[start:end]
                            seg_speed = speed[start:end]
                            seg_jerk = jerk[start:end]
                            seg_time = time[start:end]
                            seg_length_time = time[end]-time[start]
                            if seg_length_time < 0.1:
                                continue
                            max_s = np.max(seg_speed)
                            max_a = np.max(seg_accel)
                            max_j = np.max(seg_jerk)

                            area_of_speed = np.trapz(seg_speed, x=seg_time)
                            area_of_accel = np.trapz(seg_accel, x=seg_time)
                            area_of_jerk = np.trapz(seg_jerk, x=seg_time)

                            ttm_s = time[np.argmax(
                                seg_speed)+start]-time[start]
                            ttm_a = time[np.argmax(
                                seg_accel)+start]-time[start]
                            ttm_j = time[np.argmax(seg_jerk)+start]-time[start]

                            avg_s = area_of_speed/seg_length_time
                            avg_a = area_of_accel/seg_length_time
                            avg_j = area_of_jerk/seg_length_time

                            max_speed.append(max_s)
                            max_accel.append(max_a)
                            max_jerk.append(max_j)

                            avg_speed.append(avg_a)
                            avg_accel.append(avg_a)
                            avg_jerk.append(avg_j)

                            max_div_avg_speed.append(max_s/avg_s)
                            max_div_avg_accel.append(max_a/avg_a)
                            max_div_avg_jerk.append(max_j/avg_j)

                            time_to_max_speed.append(ttm_s)
                            time_to_max_speed_norm.append(
                                ttm_s/(seg_length_time))
                            time_to_max_accel.append(ttm_a)
                            time_to_max_accel_norm.append(
                                ttm_a/seg_length_time)
                            time_to_max_jerk.append(ttm_j)
                            time_to_max_jerk_norm.append(ttm_j/seg_length_time)

                            length_movement_xyz.append(area_of_speed)
                            length_movement_t.append(seg_length_time)

                            normalized_jerk.append(
                                np.sqrt(
                                    (1/2) *
                                    np.trapz(np.power(seg_jerk, 2), seg_time) *
                                    seg_length_time *
                                    area_of_speed
                                ))

                            jerk_metric.append(-avg_j/max_s)
                            speed_metric.append(avg_s/max_s)

                            completed_segment = True

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
                                np.median(max_speed),
                                np.median(max_accel),
                                np.median(max_jerk),
                                np.median(avg_speed),
                                np.median(avg_accel),
                                np.median(avg_jerk),
                                np.median(max_div_avg_speed),
                                np.median(max_div_avg_accel),
                                np.median(max_div_avg_jerk),
                                np.median(time_to_max_speed),
                                np.median(time_to_max_speed_norm),
                                np.median(time_to_max_accel),
                                np.median(time_to_max_accel_norm),
                                np.median(time_to_max_jerk),
                                np.median(time_to_max_jerk_norm),
                                np.median(length_movement_xyz),
                                np.median(length_movement_t),
                                np.median(normalized_jerk),
                                np.median(speed_metric),
                                num_movements])

    results_df = pd.DataFrame(results, columns=[
                              'subject',
                              'rep',
                              'side',
                              'bbt',
                              'age',
                              'max_speed',
                              'max_accel',
                              'max_jerk',
                              'avg_speed',
                              'avg_accel',
                              'avg_jerk',
                              'max_div_avg_speed',
                              'max_div_avg_accel',
                              'max_div_avg_jerk',
                              'time_to_max_speed',
                              'time_to_max_speed_norm',
                              'time_to_max_accel',
                              'time_to_max_accel_norm',
                              'time_to_max_jerk',
                              'time_to_max_jerk_norm',
                              'length_movement_xyz',
                              'length_movement_t',
                              'normalized_jerk',
                              'speed_metric',
                              'number_movements'])
    results_df.to_csv(target_dir/'tt_features.csv')

    # target_dir = pathlib.Path("/media/mjsobrep/43CDA61E672B9161/pose/")
if __name__ == '__main__':
    TT_PARSER = argparse.ArgumentParser()

    TT_PARSER.add_argument("-t", "--target", type=str, required=True,
                           help="where to find the hdf5 files and save the result")
    ARGS = TT_PARSER.parse_args()
    target_touch_accel(ARGS.target)
