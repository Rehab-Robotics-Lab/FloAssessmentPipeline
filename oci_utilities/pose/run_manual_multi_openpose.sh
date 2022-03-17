#!/bin/bash
set -o errexit
set -o pipefail

## This script will run openpose pose detection only. Run it on a GPU enabled machine.

bucket_hdf5='rrl-flo-hdf5'
condition='robot' # or 'podium' or 'mixed' this is the folder in oci
data_source='robot' # or 'podium'

for condition in 'robot' 'mixed' 'podium'
do
    if [ $condition = 'podium' ]
    then
        data_source='podium'
    else
        data_source='robot'
    fi

    #script="$(realpath "$0")"
    #scriptpath="$(dirname "$script")"

    [ "$(docker container ls --all --filter name=openpose-runner -q)" ] && docker rm /openpose-runner

    done_first=false
    subj_back1=''

    mkdir -p "$HOME/data"
    rm -rf "$HOME/data/"*


    for subj in "$@"
    do
        re='^[0-9]+$'
        if ! [[ "$subj" =~ $re ]] ; then
            subj="$subj"
        else
            subj=$(printf '%03d' $(( 10#$subj )))
        fi

        echo "[$(date +"%T")] Working on subject $subj, condition: $condition, source: $data_source"

        ### Figure out if subject has hdf5 files ###
        hdf5_dir="$subj/$condition"
        novid_bucket_location="$hdf5_dir/full_data-novid.hdf5"
        vid_bucket_location="$hdf5_dir/full_data-vid.hdf5"
        output_file="$hdf5_dir/full_data-novid-poses.hdf5"
        novid_present=$(oci os object list -bn "$bucket_hdf5" --prefix "$novid_bucket_location" --query 'contains(keys(@),`data`)')
        vid_present=$( oci os object list -bn "$bucket_hdf5" --prefix "$vid_bucket_location" --query 'contains(keys(@),`data`)')
        output_present=$( oci os object list -bn "$bucket_hdf5" --prefix "$output_file" --query 'contains(keys(@),`data`)')

        # If the vid and novid are not present, then skip
        if [ "$novid_present" != true ] || [ "$vid_present" != true ]
        then
            echo "[$(date +"%T")] Subject $subj is missing $condition hdf5 files !!!! No further processing will be done for the subject."
            continue
        fi

        if [ "$output_present" = true ]
        then
            echo "[$(date +"%T")] Subject $subj already has an output file generated for $condition hdf5 files !!!! No further processing will be done for the subject."
            continue
        fi

        ### Download Files ###
        subj_data_local="$HOME/data/$subj"
        subj_back1_data_local="$HOME/data/$subj_back1"
        vid_local_location="$subj_data_local/full_data-vid.hdf5"
        novid_local_location="$subj_data_local/full_data-novid.hdf5"
        mkdir -p "$subj_data_local"
        echo "[$(date +"%T")] Downloading files for: $subj"
        oci os object get -bn "$bucket_hdf5" --name "$novid_bucket_location" --file "$novid_local_location" # fast
        oci os object get -bn "$bucket_hdf5" --name "$vid_bucket_location" --file "$vid_local_location" # approx 30 minutes
        oci os object get -bn "rrl-flo-transforms" --name "transforms.json" --file "$subj_data_local/transforms.json"

        if [ $done_first = true ]
        then
            echo "[$(date +"%T")] waiting for subj $subj_back1 openpose to finish"
            docker_exit_code=$(docker container wait openpose-runner)
            echo "[$(date +"%T")] openpose finished"
            docker rm /openpose-runner
            echo "[$(date +"%T")] removed old docker instance for openpose"
            if [ "$docker_exit_code" -ne 0 ]
            then
                echo "[$(date +"%T")] error from docker exit from openpose"
                exit "$docker_exit_code"
            fi


            echo "[$(date +"%T")] starting file upload for subj $subj_back1"
            oci os object put \
                --bucket-name $bucket_hdf5 \
                --file "$subj_back1_data_local/full_data-novid.hdf5" \
                --name "$subj_back1/$condition/full_data-novid-poses.hdf5" \
                --force
            rm -rf "$subj_back1_data_local"
            echo "[$(date +"%T")] done with file upload for subj $subj_back1"

        fi

        echo "[$(date +"%T")] starting openpose run"
        docker run \
            --mount type=bind,source="$subj_data_local",target=/data \
            --detach \
            --log-driver=journald \
            --name=openpose-runner \
            openpose \
            python3 -m pose.src.process_hdf5 -d "/data/" -s "$data_source" -a 'openpose:25B' -c 'lower' -p 'pose'

            #--detach \

        subj_back1="$subj"
        done_first=true
    done

    subj_back1_data_local="$HOME/data/$subj_back1"

    if [ $done_first = true ]
    then
        echo "[$(date +"%T")] waiting for previous subj openpose to finish"
        docker_exit_code=$(docker container wait openpose-runner)
        echo "[$(date +"%T")] docker finished "
        docker rm /openpose-runner
        echo "[$(date +"%T")] removed old instance"
        if [ "$docker_exit_code" -ne 0 ]
        then
            echo "[$(date +"%T")] error from docker exit"
            exit "$docker_exit_code"
        fi

        echo "[$(date +"%T")] starting file upload for subj $subj_back1"
        oci os object put \
            --bucket-name $bucket_hdf5 \
            --file "$subj_back1_data_local/full_data-novid.hdf5" \
            --name "$subj_back1/$condition/full_data-novid-poses.hdf5" \
            --force
        rm -rf "$subj_back1_data_local"
        echo "[$(date +"%T")] done with file upload for subj $subj_back1"
    fi

    echo "[$(date +"%T")] done with pose processing"

    rm -rf "$HOME/data/*"

done

echo "--------------------------------------------"
echo "--- Done running pose detection !!!!!!! ----"
echo "--------------------------------------------"
