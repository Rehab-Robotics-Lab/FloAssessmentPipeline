"""Data loader helper functions"""
import h5py
import pandas as pd
import pathlib


def load_game_data(target_dir, dset):
    """ Loads the game data """
    target_dir = pathlib.Path(target_dir)

    hdf5_file = h5py.File(target_dir/"smoothed_data.hdf5", 'r')
    data_set = pd.read_csv(target_dir/f"{dset}.csv")
