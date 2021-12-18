#!/bin/bash
set -o errexit
set -o pipefail

bucket_raw='rrl-flo-raw'
bucket_hdf5='rrl-flo-hdf5'

script="$(realpath "$0")"
scriptpath="$(dirname "$script")"

# shellcheck source=../includes/parse_input_subj_no.sh
source "$scriptpath/../includes/parse_input_subj_no.sh"

#Get the ros files of interest
echo 'looking for mixed files (old style)'
# Don't waant the query string expanded
# shellcheck disable=SC2016
# will be expanded using declare below
# shellcheck disable=SC2034
mixed=$(oci os object list --bucket-name "$bucket_raw" --fields name --prefix "$subject_padded" --query 'data[? ends_with("name",`"bag.bz2"`) && ! contains("name",`"podium"`) && ! contains("name",`"robot"`)] | [].name' --raw-output --all | jq '.[]' -r)

echo 'looking for podium files'
# Don't waant the query string expanded
# shellcheck disable=SC2016
# will be expanded using declare below
# shellcheck disable=SC2034
podium=$(oci os object list --bucket-name "$bucket_raw" --fields name --prefix "$subject_padded" --query 'data[? ends_with("name",`"bag.bz2"`) && contains("name",`"podium"`)] | [].name' --raw-output --all | jq '.[]' -r)

echo 'looking for robot files'
# Don't waant the query string expanded
# shellcheck disable=SC2016
# will be expanded using declare below
# shellcheck disable=SC2034
robot=$(oci os object list --bucket-name "$bucket_raw" --fields name --prefix "$subject_padded" --query 'data[? ends_with("name",`"bag.bz2"`) && contains("name",`"robot"`)] | [].name' --raw-output --all | jq '.[]' -r)

[ "$(docker container ls --all --filter name=hdf5-converter -q)" ] && docker rm /hdf5-converter

for group in mixed podium robot
do
    echo "working on $group"
    declare -n group_files="$group"

    done_first=false
    previous_bag_fn=''
    mkdir -p "$HOME/data/$group"
    rm -rf "$HOME/data/"
    mkdir -p "$HOME/data/$group"

    for bag_fn in $group_files
    do
        bag_basename=$(basename $bag_fn)
        #bring in the bag file from s3
        echo "downloading $bag_basename"
        oci os object get \
            -bn "$bucket_raw" \
            --file "$HOME/data/$group/$bag_basename" \
            --name "$bag_fn"

        echo "uncompressing"
        lbzip2 -d "$HOME/data/$group/$bag_basename"

        if [ $done_first = true ]
        then
            echo "waiting for previous file to finish"
            hdf_exit_code=$(docker container wait hdf5-converter)
            docker rm /hdf5-converter
            if [ "$hdf_exit_code" -ne 0 ]
            then
                exit "$hdf_exit_code"
            fi
            rm "$HOME/data/$group/${previous_bag_fn%.*}"
        fi

        echo "starting hdf5 transfer"
        docker run \
            --mount type=bind,source="$HOME/data/$group/",target=/data \
            --detach \
            --name=hdf5-converter \
            hdf5convert \
            roslaunch convert_to_hdf5 convert_to_hdf5.launch bag_file:="/data/${bag_basename%.*}" out_dir:="/data"


        previous_bag_fn="$bag_basename"
        done_first=true
    done

    if [ $done_first = true ]
    then
        echo "waiting for last file to finish"
        hdf_exit_code=$(docker container wait hdf5-converter)
        docker rm /hdf5-converter
        if [ "$hdf_exit_code" -ne 0 ]
        then
            exit "$hdf_exit_code"
        fi
        rm "$HOME/data/$group/${previous_bag_fn%.*}"
    fi

    echo 'done with HDF File creation, begining upload'

    files2transfer=$(find "$HOME/data/$group/" -type f -printf '%P\n')
    for fn in $files2transfer
    do
        oci os object put \
            --bucket-name $bucket_hdf5 \
            --file "$HOME/data/$group/$fn" \
            --name "$subject_padded/$group/$fn" \
            --force
    done
    echo "Done with upload"
    rm -rf "$HOME/data/*"
done


rm -rf "$HOME/data/*"


echo "--------------------------------------"
echo "--- Done generating HDF5 files !!!!---"
echo "--------------------------------------"
