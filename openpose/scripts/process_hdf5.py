#!/usr/bin/env python3
"""Module to extract pose using openpose from hdf5 file"""

import pathlib
import sys
import numpy as np
import h5py
from extract_poses import process_frames
from extract_depth import add_stereo_depth
from tqdm import tqdm


def allkeys(obj):
    "Recursively find all datasets in an h5py.Group."
    keys = (obj.name,)
    if isinstance(obj, h5py.Group):
        for _, value in obj.items():
            if isinstance(value, h5py.Group):
                keys = keys + allkeys(value)
            else:
                keys = keys + (value.name,)
    return keys


def convert(pth):
    # pylint: disable= too-many-statements
    """Extract poses from video in hdf5 file and build new hdf
    file with poses, confidences, and other non-video data.

    Args:
        pth: the path of the hdf5 source file to work with
    """
    print('processing on {}'.format(pth))

    # open hdf5 file
    try:
        hdf5_in = h5py.File(pth, 'r+')
    except:  # pylint: disable=bare-except
        print('HDF5 database could not be read')
        raise

    print('opened hdf file')

    added_keypoints = False

    nodes = allkeys(hdf5_in)
    print('Parsing file')
    for dset in tqdm(nodes, desc='datasets'):
        tqdm.write('\t{}'.format(dset))
        if 'color' in dset and 'lower' in dset and isinstance(hdf5_in[dset], h5py.Dataset):
            tqdm.write('\t\tVideo, starting processing')

            if dset+'-keypoints' not in hdf5_in:
                keypoints_dset = hdf5_in.create_dataset(
                    dset+'-keypoints', (hdf5_in[dset].len(), 25, 2), dtype=np.float32)
            else:
                keypoints_dset = hdf5_in[dset+'-keypoints']

            if dset+'-confidence' not in hdf5_in:
                confidence_dset = hdf5_in.create_dataset(
                    dset+'-confidence', (hdf5_in[dset].len(), 25), dtype=np.float32)
            else:
                confidence_dset = hdf5_in[dset+'-confidence']

            for chunk in tqdm(hdf5_in[dset].iter_chunks(), desc='chunks'):
                color_arr = hdf5_in[dset][chunk]
                keypoints = process_frames(color_arr)
                keypoints_dset[chunk[0], :, :] = keypoints[:, :, 0:2]
                confidence_dset[chunk[0], :] = keypoints[:, :, 2]

            print('Adding Stereo Depth')
            add_stereo_depth(hdf5_in, dset)
            print('Done Adding Stereo Depth')

    print('done processing')
    hdf5_in.close()
    print('done closing')


if __name__ == '__main__':
    convert(sys.argv[1])
