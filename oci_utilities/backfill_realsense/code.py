import h5py
from pathlib import Path


f = h5py.File(f'{Path.home()}/data/full_data.hdf5')

if f['vid'] and f['vid']['depth_to_color']:
    for cam in ('upper', 'lower'):
        if f['vid']['depth_to_color'][cam] and f['vid']['depth_to_color'][cam]['data']:
            data = f['vid']['depth_to_color']['lower']['data']
            names = data.dtype.names

