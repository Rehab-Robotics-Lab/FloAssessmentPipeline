"""
A module for going through all of the hdf5 files and getting all 
of the available transformations between the epth aand color imagers. 
"""
#!/usr/bin/env python3
import pathlib
import argparse
import json
import h5py


def get_transforms(out_dir):
    """Copy all non-video data from

    Args:
        out_dir:
    """
    path = pathlib.Path(out_dir)
    transforms = {
        'robot': {'upper': {}, 'lower': {}},
        'podium': {'upper': {}, 'lower': {}},
        'mixed': {'upper': {}, 'lower': {}}
    }

    print('Searching in path: {path}')
    for hdf5_fn in path.rglob('*novid.hdf5'):
        print(f'Working on: {hdf5_fn}')
        try:
            hdf5_database = h5py.File(hdf5_fn, 'r')
        except:  # pylint: disable=bare-except
            print(
                'HDF5 Database COULD NOT BE READ/CREATED: %s', hdf5_fn)
        source = hdf5_fn.parts[-2]  # robot or podium or mixed
        for cam in ('lower', 'upper'):
            if (f'vid/depth_to_color/{cam}/time' in hdf5_database and
                    f'vid/depth_to_color/{cam}/data' in hdf5_database):
                print('found transforms')
                for time, data in zip(
                        hdf5_database[f'vid/depth_to_color/{cam}/time'],
                        hdf5_database[f'vid/depth_to_color/{cam}/data']):
                    transforms[source][cam][time] = {
                        'rotation': data[0].tolist(),
                        'translation': data[1].tolist()}

        hdf5_database.close()
    with open(path / 'transforms.json', 'w', encoding="utf8") as file_obj:
        json.dump(transforms, file_obj)


if __name__ == '__main__':
    PARSER = argparse.ArgumentParser()

    PARSER.add_argument("-t", "--target", type=str, required=True,
                        help="where to find the hdf5 files and save the result")
    ARGS = PARSER.parse_args()
    get_transforms(ARGS.target)
