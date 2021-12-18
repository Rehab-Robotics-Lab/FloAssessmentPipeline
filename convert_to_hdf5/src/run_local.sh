#!/bin/bash
set -o errexit
set -o pipefail

# parse options
while getopts :hd: flag
do
    case "${flag}" in
        d) dir=${OPTARG};;
        h) echo "pass in directory with bag files under tag -d, will output hdf5 files to the same location"; exit 0;;
        :) echo 'missing argument' >&2; exit 1;;
        \?) echo 'invalid option' >&2; exit 1
    esac
done

if [ -z "$dir" ]
then
    echo "Missing target directory argument"
    exit 1
fi

dir=${dir%/}

echo "looking for files at: $dir"
echo "---------------"

bag_files=$(ls "$dir/"*'.bag')
echo "processing on: "
echo "$bag_files"
echo "---------------"
echo "---------------"


for bag_fn in $bag_files
do
    bag_fn=${bag_fn#"$dir"}
    bag_fn=${bag_fn#"/"}
    echo "working on $bag_fn"
    docker run \
        --mount type=bind,source="$dir",target=/data \
        -it \
        --rm \
        --name=hdf5-converter \
        hdf5convert \
        roslaunch convert_to_hdf5 convert_to_hdf5.launch bag_file:="/data/${bag_fn}" out_dir:="/data"
    echo "done with file"

done


echo "--------------------------------------"
echo "--- Done generating HDF5 files !!!!---"
echo "--------------------------------------"
