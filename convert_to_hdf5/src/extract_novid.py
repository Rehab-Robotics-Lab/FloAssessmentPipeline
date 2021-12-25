"""
Module for splitting out an HDF5 file with all of the data except the video
"""

#!/usr/bin/env python3
import pathlib
import argparse
import h5py


VIDEO_TOPICS = [
    'vid/color/lower/data',
    'vid/color/upper/data',
    'vid/depth/lower/data',
    'vid/depth/upper/data'
]


def load_hdf_file(out_dir, appendix, mode):
    """Load HDF file, creating it if needed

    Args:
        out_dir: The directory to save to
        appendix: The string to append to data

    Returns: The HDF5 File data object
    """
    hdf5_fn_full = pathlib.Path(
        out_dir, f'full_data-{appendix}').with_suffix('.hdf5')
    pth = pathlib.Path(hdf5_fn_full)
    pth.parent.mkdir(parents=True, exist_ok=True)
    try:
        hdf5_database = h5py.File(hdf5_fn_full, mode)
    except:  # pylint: disable=bare-except
        print(
            'HDF5 Database COULD NOT BE READ/CREATED: %s', hdf5_fn_full)
        raise
    return hdf5_database


def copy_vid_to_novid(out_dir):
    """Copy all non-video data from

    Args:
        out_dir:
    """
    print('creating split out hdf5 file')
    hdf5_file = load_hdf_file(out_dir, 'vid', 'r')
    # w-: write, fail if file exists
    hdf5_file_novid = load_hdf_file(out_dir, 'novid', 'w-')

    def place_dset(dset_name, dset_data):
        if not isinstance(dset_data, h5py.Dataset):
            return
        if dset_name not in VIDEO_TOPICS:
            group = '/'.join(dset_name.split('/')[0:-1])
            hdf5_file_novid.require_group(group)
            hdf5_file.copy(dset_data, hdf5_file_novid[group])

    hdf5_file.visititems(place_dset)

    hdf5_file.close()
    hdf5_file_novid.close()


if __name__ == '__main__':
    PARSER = argparse.ArgumentParser()

    PARSER.add_argument("-t", "--target", type=str,
                        help="where to find and save the hdf5_file.")
    ARGS = PARSER.parse_args()
    copy_vid_to_novid(ARGS.target)
