import numpy as np
import h5py
import pathlib
from matplotlib import pyplot as plt
from scipy import signal
import json
import hampel
import pykalman
from mpl_toolkits import mplot3d
from pose.src import joints


def load_hdf5_files(data_folder):
    hdf5_files = {}
    file_names = {}
    data_folder = pathlib.Path(data_folder)
    for subj_folder in data_folder.iterdir():
        if not subj_folder.is_dir():
            continue
        subj_no = subj_folder.parts[-1]
        file_name = subj_folder/'robot'/'full_data-novid-poses-depth.hdf5'
        if (file_name).exists():
            hdf5_files[subj_no] = h5py.File(file_name, 'r')
            file_names[subj_no] = file_name
        file_name = subj_folder/'mixed'/'full_data-novid-poses-depth.hdf5'
        if (file_name).exists():
            hdf5_files[subj_no] = h5py.File(file_name, 'r')
            file_names[subj_no] = file_name
        # note, subjects without data are just not added to dictionary
    return (hdf5_files, file_names)


def check_data(hdf5_files, cutoff=2000):
    for subj in hdf5_files:
        if (len(
            hdf5_files[subj]['vid/lower/pose/openpose:25B/keypoints-median5/3d-realsense-raw']
        ) < cutoff):
            print(f'bad data for {subj} lower camera openpose')
        if (len(
            hdf5_files[subj]['vid/upper/pose/mp-hands/left/keypoints-median5/3d-realsense-raw']
        ) < cutoff):
            print(f'bad data for {subj} upper camera mp-hands left hand')
        if (len(
            hdf5_files[subj]['vid/upper/pose/mp-hands/right/keypoints-median5/3d-realsense-raw']
        ) < cutoff):
            print(f'bad data for {subj} upper camera mp-hands right hand')


def print_members(hdf5_file):
    def print_attrs(name, _):
        print(name)
    hdf5_file.visititems(print_attrs)
