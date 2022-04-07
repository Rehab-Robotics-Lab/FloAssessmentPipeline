import scipy
from post_processing.src import hdf5_tools
from post_processing.src import plotting
from post_processing.src import filter_joints
import matplotlib.pyplot as plt
import pathlib
import multiprocessing
import h5py
import ipdb
import numpy as np


def filter_joints_wrapper(args):
    idx, fn = args
    hdf5_file = h5py.File(fn, 'r')
    return idx, filter_joints.filter_file(hdf5_file)


def smooth_data(path_name):
    target_dir = pathlib.Path(path_name)
    _, hdf5_files_names = hdf5_tools.load_hdf5_files(target_dir)
    hdf5_out = h5py.File(target_dir/'smoothed_data.hdf5', 'w-')

    all_subj = list(hdf5_files_names.keys())
    # all_subj.remove('060')
    # all_subj.remove('500-3')

    files_list = [(subj, hdf5_files_names[subj]) for subj in all_subj]
    with multiprocessing.Pool() as pool:
        for subj_id, result in pool.imap(filter_joints_wrapper, files_list):
            # for subj_id, result in map(filter_joints_wrapper, files_list):
            print(f'inserting data for {subj_id}')
            game_counter = {}
            for game in result:
                game_type = game['game_type']
                if game_type not in game_counter:
                    game_counter[game_type] = 0
                else:
                    game_counter[game_type] += 1
                if 'state' in game and game['state'] is not None:
                    hdf5_out.create_dataset(
                        f'{subj_id}/{game_type}/{game_counter[game_type]}/time', data=game['state']['time'])
                    for group in ['smooth', 'raw', 'filtered', 'covariance']:
                        for joint in ['RWrist', 'RElbow', 'RShoulder', 'LWrist', 'LElbow', 'LShoulder']:
                            if joint in game['state'][group]:
                                hdf5_out.create_dataset(
                                    f'{subj_id}/{game_type}/{game_counter[game_type]}/{group}/{joint}',
                                    data=game['state'][group][joint])


if __name__ == '__main__':
    path_name = "/media/mjsobrep/43CDA61E672B9161/pose"
    with ipdb.launch_ipdb_on_exception():
        smooth_data(path_name)
