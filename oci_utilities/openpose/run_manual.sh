#!/bin/bash
set -o errexit
set -o pipefail

bucket_hdf5='rrl-flo-hdf5'

script="$(realpath "$0")"
scriptpath="$(dirname "$script")"

# shellcheck source=../includes/parse_input_subj_no.sh
source "$scriptpath/../includes/parse_input_subj_no.sh"

#Get the ros files of interest
echo 'looking for hdf5 files'
# Don't waant the query string expanded
# shellcheck disable=SC2016
# will be expanded using declare below
# shellcheck disable=SC2034
hdf_files=$(oci os object list --bucket-name "$bucket_hdf5" --fields name --prefix "$subject_padded" --query 'data[? ends_with("name",`".hdf5"`)] | [].name' --raw-output --all | jq '.[]' -r)

echo "Found hdf files:"
echo "$hdf_files"


[ "$(docker container ls --all --filter name=openpose-runner -q)" ] && docker rm /openpose-runner

done_first=false
previous_fn=''
previous_group=''
rm -rf "$HOME/data/"*

for fn in $hdf_files
do
    fn_basename=$(basename "$fn")
    group=$(basename "${fn%'/full_data.hdf5'}")
    mkdir -p "$HOME/data/$group"
    #bring in the bag file from s3
    echo "downloading $fn"
    oci os object get \
        -bn "$bucket_hdf5" \
        --file "$HOME/data/$group/$fn_basename" \
        --name "$fn"

    if [ $done_first = true ]
    then
        echo "waiting for previous file to finish"
        docker_exit_code=$(docker container wait openpose-runner)
        echo "done waiting"
        docker rm /openpose-runner
        echo "removed old instance"
        if [ "$docker_exit_code" -ne 0 ]
        then
            echo "error from docker exit"
            exit "$docker_exit_code"
        fi
        echo "starting file upload"
        oci os object put \
            --bucket-name $bucket_hdf5 \
            --file "$HOME/data/$previous_group/$previous_fn" \
            --name "$subject_padded/$previous_group/$previous_fn" \
            --force &
    fi

    echo "starting openpose run"
    docker run \
        --mount type=bind,source="$HOME/data/$group/",target=/data \
        --name=openpose-runner \
        openpose \
        ./process_hdf5.py "/data/$fn_basename"

        #--detach \

    previous_fn="$fn_basename"
    previous_group="$group"
    done_first=true
done

if [ $done_first = true ]
then
    echo "waiting for last file to finish"
    docker_exit_code=$(docker container wait openpose-runner)
    docker rm /openpose-runner
    if [ "$docker_exit_code" -ne 0 ]
    then
        exit "$docker_exit_code"
    fi
    oci os object put \
        --bucket-name $bucket_hdf5 \
        --file "$HOME/data/$previous_group/$previous_fn" \
        --name "$subject_padded/$previous_group/$previous_fn" \
        --force # &
fi

echo 'done with openpose processing, waiting for upload'
wait

echo "Done with upload"
rm -rf "$HOME/data/*"


echo "--------------------------------------"
echo "--- Done running openpose !!!!!!! ----"
echo "--------------------------------------"
