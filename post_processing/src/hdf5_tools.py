"""Tools for working with HDF5 files produced by pose estimation/depth extraction"""

import pathlib
import h5py


def load_hdf5_files(data_folder):
    """Given a directory structure of:

    data_folder -> subj number -> source (robot/mixed) ->
    full_data-novid-poses-depth.hdf5

    Opens each hdf5 file found (for each subject) read only.

    Note: Does not open files from podium.

    Args:
        data_folder: The path to the data folder that holds all
                     of the HDF5 files
    Return: (hdf5 files (opened), hdf5 filenames)
    """
    hdf5_files = {}
    file_names = {}
    data_folder = pathlib.Path(data_folder)
    for subj_folder in data_folder.iterdir():
        if not subj_folder.is_dir():
            continue
        subj_no = subj_folder.parts[-1]
        file_name = subj_folder/'robot'/'full_data-novid-poses-depth.hdf5'
        found = False
        if (file_name).exists():
            hdf5_files[subj_no] = h5py.File(file_name, 'r')
            file_names[subj_no] = file_name
            found = True
        file_name = subj_folder/'mixed'/'full_data-novid-poses-depth.hdf5'
        if (file_name).exists():
            hdf5_files[subj_no] = h5py.File(file_name, 'r')
            file_names[subj_no] = file_name
            if found:
                raise RuntimeError('Both robot and mixed data found')
        # note, subjects without data are just not added to dictionary
    return (hdf5_files, file_names)


def check_data(hdf5_files, cutoff=2000):
    """Checks to see if any subjects in a group of HDF5 files are missing data

    checks whether the lower camera has enough openpose frames and whether the
    upper camera has enough pose detections from media pipe for both the left and
    right arms.

    Args:
        hdf5_files: Dictionary of pose files, the keys should be subject IDs
        cutoff: The minimum number of data points allowable to consider a data set OK
    """
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
    """Traverses a HDF5 file, printing all of its members (groups and datasets)

    Args:
        hdf5_file: The file to traverse
    """
    def print_attrs(name, _):
        print(name)
    hdf5_file.visititems(print_attrs)
