# Convet To HDF5

This pakcage converts bag files from the Lil'Flo platform to HDF5 files.

## Running

### With Docker

Temporary:

1.  Put a bag file named `in.bag` into a data folder somewhere
2.  Run `test.sh <path to data folder>` from within this directory
3.  Your HDF5 file will now be in in your data folder

### EC2

1.  Setup an instance of type c5ad.12xlarge
2.  Install curl and unzip
3.  Install aws cli (make sure to get x86 version)
4.  install lbzip2
5.  on every start, run `convert_to_hdf5/mount_instance_store.sh`
6.  build docker file: `docker build . --tag hdf5convert`
7.  run `./convert_to_hdf5/src/process-in-ec2.sh`
