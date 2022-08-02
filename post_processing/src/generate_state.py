"""Module for ingesting HDF5 files withe game and pose data and generating
an hdf5 file with resulting"""
import argparse
import multiprocessing
import pathlib
import h5py
from post_processing.src import hdf5_tools
from post_processing.src import filter_joints


def filter_joints_wrapper(args):
    """A wrapper around post_processing.src.filter_joints to support
    multiprocessing.

    Args:
        args: tupple with index that should be returned and hdf5 filename
    """
    idx, fn = args
    hdf5_file = h5py.File(fn, 'r')
    return idx, filter_joints.filter_file(hdf5_file)


def smooth_data(path_name):
    """Process pose data in games, stored in hdf5 files inside of a
    directory, put all results into a single hdf5 file.

    Expects the data to be stored under:
    path_name -> subj number -> source (robot/mixed) ->
    full_data-novid-poses-depth.hdf5

    Creates a HDF5 file `path_name/smoothed_data.hdf5`

    Args:
        path_name: The path to look for files in
    """
    #pylint: disable=too-many-locals
    #pylint: disable=too-many-branches
    target_dir = pathlib.Path(path_name)
    _, hdf5_files_names = hdf5_tools.load_hdf5_files(target_dir)
    hdf5_out = h5py.File(target_dir/'smoothed_data.hdf5', 'w-')

    all_subj = list(hdf5_files_names.keys())

    files_list = [(subj, hdf5_files_names[subj]) for subj in all_subj]
    with multiprocessing.Pool() as pool:
        # pylint: disable=too-many-nested-blocks
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
                if game_type == 'simon_says' and 'state' in game and game['state'] is not None:
                    hdf5_out.create_dataset(
                        f'{subj_id}/{game_type}/{game_counter[game_type]}/time',
                        data=game['state']['time'])
                    for group in ['smooth', 'raw', 'filtered', 'covariance']:
                        for joint in [
                                'RWrist', 'RElbow', 'RShoulder', 'LWrist', 'LElbow', 'LShoulder']:
                            if joint in game['state'][group]:
                                hdf5_out.create_dataset(
                                    f'{subj_id}/{game_type}/' +
                                    f'{game_counter[game_type]}/{group}/{joint}',
                                    data=game['state'][group][joint])
                if game_type == 'target_touch':
                    time_added = False
                    for arm in ['left', 'right']:
                        if arm not in game['state'] or game['state'][arm] is None:
                            continue
                        if not time_added:
                            hdf5_out.create_dataset(
                                f'{subj_id}/{game_type}/{game_counter[game_type]}/time',
                                data=game['state'][arm]['time'])
                            time_added = True
                        for group in ['smooth', 'raw', 'filtered', 'covariance']:
                            hdf5_out.create_dataset(
                                f'{subj_id}/{game_type}/' +
                                f'{game_counter[game_type]}/{group}/{arm}',
                                data=game['state'][arm][group])


if __name__ == '__main__':
    SMOOTH_PARSER = argparse.ArgumentParser()

    SMOOTH_PARSER.add_argument("-t", "--target", type=str, required=True,
                               help="where to find the hdf5 files and save the result")
    ARGS = SMOOTH_PARSER.parse_args()
    # with ipdb.launch_ipdb_on_exception():
    smooth_data(ARGS.target)
