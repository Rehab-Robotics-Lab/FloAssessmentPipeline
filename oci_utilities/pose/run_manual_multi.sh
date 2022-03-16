#!/bin/bash
set -o errexit
set -o pipefail

## NOTE: the list of subjects must be passed in perfectly formatted

bucket_hdf5='rrl-flo-hdf5'
condition='robot' # or 'podium' or maybe 'mixed'

#script="$(realpath "$0")"
#scriptpath="$(dirname "$script")"

[ "$(docker container ls --all --filter name=openpose-runner -q)" ] && docker rm /openpose-runner
[ "$(docker container ls --all --filter name=mediapipe-runner -q)" ] && docker rm /mediapipe-runner

done_first=false
done_second=false
subj_back1=''
subj_back2=''

mkdir -p "$HOME/data"
rm -rf "$HOME/data/"*


for subj in "$@"
do
    ### Figure out if subject has hdf5 files ###
    bag_dir="$subj/$condition"
    novid_bucket_location="$bag_dir/full_data-novid.hdf5"
    vid_bucket_location="$bag_dir/full_data-vid.hdf5"
    novid_present=$(oci os object list -bn "$bucket_hdf5" --prefix "$novid_bucket_location" --query 'data|length(@)')
    vid_present=$(oci os object list -bn "$bucket_hdf5" --prefix "$vid_bucket_location" --query 'data|length(@)')

    # If the vid and novid are not present, then skip
    if [ "$novid_present" != 1 ] || [ "$vid_present" != 1 ]
    then
        echo "Subject $subj is missihg $condition hdf5 files !!!! No further processing will be done for the subject."
        continue
    fi

    ### Download Files ###
    subj_data_local="$HOME/data/$subj"
    subj_back1_data_local="$HOME/data/$subj_back1"
    subj_back2_data_local="$HOME/data/$subj_back2"
    vid_local_location="$subj_data_local/full_data-vid.hdf5"
    novid_local_location="$subj_data_local/full_data-novid.hdf5"
    mkdir -p "$subj_data_local"
    oci os object get -bn "$bucket_hdf5" --name "$novid_bucket_location" --file "$novid_local_location" # fast
    oci os object get -bn "$bucket_hdf5" --name "$vid_bucket_location" --file "$vid_local_location" # approx 30 minutes
    oci os object get -bn "rrl-flo-transforms" --name "transforms.json" --file "$subj_data_local/transforms.json"

    if [ $done_first = true ]
    then
        echo "waiting for previous subj openpose to finish"
        docker_exit_code=$(docker container wait openpose-runner)
        echo "docker finished "
        docker rm /openpose-runner
        echo "removed old instance"
        if [ "$docker_exit_code" -ne 0 ]
        then
            echo "error from docker exit"
            exit "$docker_exit_code"
        fi

        if [ $done_second = true ]
        then
            echo "waiting for previous subj mediapipe-hands to finish"
            docker_exit_code=$(docker container wait mediapipe-runner)
            echo "docker finished "
            docker rm /mediapipe-runner
            echo "removed old instance"
            if [ "$docker_exit_code" -ne 0 ]
            then
                echo "error from docker exit"
                exit "$docker_exit_code"
            fi

            echo "starting file upload"
            oci os object put \
                --bucket-name $bucket_hdf5 \
                --file "$subj_back2_data_local/full_data-novid.hdf5" \
                --name "$subj_back2/$condition/full_data-novid-poses.hdf5" \
                --force
            rm -rf "$subj_back2_data_local"
        fi

        docker run \
            --mount type=bind,source="$subj_back1_data_local",target=/data \
            --name=mediapipe-runner \
            --detach \
            mediapipe \
            python3 -m pose.src.process_hdf5 -d "/data/" -s "$condition" -a 'mp-hands' -c 'upper'

            done_second=true
    fi

    echo "starting openpose run"
    docker run \
        --mount type=bind,source="$subj_data_local",target=/data \
        --detach \
        --name=openpose-runner \
        openpose \
        python3 -m pose.src.process_hdf5 -d "/data/" -s "$condition" -a 'openpose:25B' -c 'lower'

        #--detach \

    subj_back2="$subj_back1"
    subj_back1="$subj"
    done_first=true
done

if [ $done_first = true ]
then
    echo "waiting for previous subj openpose to finish"
    docker_exit_code=$(docker container wait openpose-runner)
    echo "docker finished "
    docker rm /openpose-runner
    echo "removed old instance"
    if [ "$docker_exit_code" -ne 0 ]
    then
        echo "error from docker exit"
        exit "$docker_exit_code"
    fi

    if [ $done_second = true ]
    then
        echo "waiting for previous subj mediapipe-hands to finish"
        docker_exit_code=$(docker container wait mediapipe-runner)
        echo "docker finished "
        docker rm /mediapipe-runner
        echo "removed old instance"
        if [ "$docker_exit_code" -ne 0 ]
        then
            echo "error from docker exit"
            exit "$docker_exit_code"
        fi

        echo "starting file upload"
        oci os object put \
            --bucket-name $bucket_hdf5 \
            --file "$subj_back2_data_local/full_data-novid.hdf5" \
            --name "$subj_back2/$condition/full_data-novid-poses.hdf5" \
            --force
        rm -rf "$subj_back2_data_local"
    fi

    docker run \
        --mount type=bind,source="$subj_back1_data_local",target=/data \
        --name=mediapipe-runner \
        mediapipe \
        python3 -m pose.src.process_hdf5 -d "/data/" -s "$condition" -a 'mp-hands' -c 'upper'

    oci os object put \
        --bucket-name $bucket_hdf5 \
        --file "$subj_back1_data_local/full_data-novid.hdf5" \
        --name "$subj_back1/$condition/full_data-novid-poses.hdf5" \
        --force
    rm -rf "$subj_back1_data_local"
fi

echo 'done with pose processing'

rm -rf "$HOME/data/*"


echo "--------------------------------------------"
echo "--- Done running pose detection !!!!!!! ----"
echo "--------------------------------------------"
