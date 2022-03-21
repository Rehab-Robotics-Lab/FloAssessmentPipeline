#!/bin/bash
set -o errexit
set -o pipefail

## Runs the hand detector and depth extraction assumes that openpose as already been run and the
## data has been pushed back up to

bucket_hdf5='rrl-flo-hdf5'
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

    [ "$(docker container ls --all --filter name=mediapipe-runner -q)" ] && docker rm /mediapipe-runner
    [ "$(docker container ls --all --filter name=openpose-runner -q)" ] && docker rm /openpose-runner

    done_first=false
    done_second=false
    subj_back1=''
    subj_back2=''

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

        novid_bucket_location="$hdf5_dir/full_data-novid-poses.hdf5" # here looking for the file that was aready passed through openpose
        vid_bucket_location="$hdf5_dir/full_data-vid.hdf5"
        output_file="$hdf5_dir/full_data-novid-poses-depth.hdf5"

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
        subj_back2_data_local="$HOME/data/$subj_back2"
        vid_local_location="$subj_data_local/full_data-vid.hdf5"
        novid_local_location="$subj_data_local/full_data-novid.hdf5" # we strip of the pose bit to work with the pipeline

        mkdir -p "$subj_data_local"
        echo "[$(date +"%T")] Downloading files for: $subj"
        oci os object get -bn "$bucket_hdf5" --name "$novid_bucket_location" --file "$novid_local_location" # fast
        oci os object get -bn "$bucket_hdf5" --name "$vid_bucket_location" --file "$vid_local_location" # approx 30 minutes
        oci os object get -bn "rrl-flo-transforms" --name "transforms.json" --file "$subj_data_local/transforms.json"

        if [ $done_first = true ]
        then
            echo "[$(date +"%T")] waiting for subj $subj_back1 mp-hands to finish"
            docker_exit_code=$(docker container wait mediapipe-runner)
            echo "[$(date +"%T")] mp-hands finished"
            docker rm /mediapipe-runner
            echo "[$(date +"%T")] removed old docker instance for mp-hands"
            if [ "$docker_exit_code" -ne 0 ]
            then
                echo "[$(date +"%T")] error from docker exit from mp-hands"
                exit "$docker_exit_code"
            fi

            if [ $done_second = true ]
            then
                echo "[$(date +"%T")] waiting for subj $subj_back2 openpose-depth to finish"
                docker_exit_code=$(docker container wait openpose-runner)
                echo "[$(date +"%T")] openpose depth finished "
                docker rm /openpose-runner
                echo "[$(date +"%T")] removed old docker instance for openpose depth"
                if [ "$docker_exit_code" -ne 0 ]
                then
                    echo "[$(date +"%T")] error from docker exit for openpose depth"
                    exit "$docker_exit_code"
                fi

                echo "[$(date +"%T")] starting file upload from $subj_back2_data_local"
                oci os object put \
                    --bucket-name $bucket_hdf5 \
                    --file "$subj_back2_data_local/full_data-novid.hdf5" \
                    --name "$subj_back2/$condition/full_data-novid-poses-depth.hdf5" \
                    --force
                rm -rf "$subj_back2_data_local"
            fi

            echo "[$(date +"%T")] starting openpose depth detection for subj $subj_back1"
            # no need for all of the openpose stuff here, just using mediapipe image
            docker run \
                --mount type=bind,source="$subj_back1_data_local",target=/data \
                --name=openpose-runner \
                --cpus="$(nproc)" \
                --log-driver=journald \
                --detach \
                mediapipe \
                python3 -m pose.src.process_hdf5 -d "/data/" -s "$data_source" -a 'openpose:25B' -c 'lower' -p 'depth'

                done_second=true
        fi

        echo "[$(date +"%T")] starting mediapipe run"
        docker run \
            --mount type=bind,source="$subj_data_local",target=/data \
            --detach \
            --cpus="$(nproc)" \
            --log-driver=journald \
            --name=mediapipe-runner \
            mediapipe \
            python3 -m pose.src.process_hdf5 -d "/data/" -s "$data_source" -a 'mp-hands' -c 'upper' -p 'pose' -p 'depth'

            #--detach \

        subj_back2="$subj_back1"
        subj_back1="$subj"
        done_first=true
    done

    subj_back1_data_local="$HOME/data/$subj_back1"
    subj_back2_data_local="$HOME/data/$subj_back2"

    if [ $done_first = true ]
    then
        echo "[$(date +"%T")] waiting for subj $subj_back1 mp-hands to finish"
        docker_exit_code=$(docker container wait mediapipe-runner)
        echo "[$(date +"%T")] mp-hands finished"
        docker rm /mediapipe-runner
        echo "[$(date +"%T")] removed old docker instance for mp-hands"
        if [ "$docker_exit_code" -ne 0 ]
        then
            echo "[$(date +"%T")] error from docker exit from mp-hands"
            exit "$docker_exit_code"
        fi

        if [ $done_second = true ]
        then
            echo "[$(date +"%T")] waiting for subj $subj_back2 openpose-depth to finish"
            docker_exit_code=$(docker container wait openpose-runner)
            echo "[$(date +"%T")] openpose depth finished "
            docker rm /openpose-runner
            echo "[$(date +"%T")] removed old docker instance for openpose depth"
            if [ "$docker_exit_code" -ne 0 ]
            then
                echo "[$(date +"%T")] error from docker exit for openpose depth"
                exit "$docker_exit_code"
            fi

            echo "[$(date +"%T")] starting file upload"
            oci os object put \
                --bucket-name $bucket_hdf5 \
                --file "$subj_back2_data_local/full_data-novid.hdf5" \
                --name "$subj_back2/$condition/full_data-novid-poses-depth.hdf5" \
                --force
            rm -rf "$subj_back2_data_local"
        fi

        echo "[$(date +"%T")] starting openpose depth detection for subj $subj_back1"
        docker run \
            --mount type=bind,source="$subj_back1_data_local",target=/data \
            --name=openpose-runner \
            --cpus="$(nproc)" \
            --log-driver=journald \
            --detach \
            mediapipe \
            python3 -m pose.src.process_hdf5 -d "/data/" -s "$data_source" -a 'openpose:25B' -c 'lower' -p 'depth'

            done_second=true

            echo "[$(date +"%T")] waiting for subj $subj_back1 openpose-depth to finish"
            docker_exit_code=$(docker container wait openpose-runner)
            echo "[$(date +"%T")] openpose depth finished "
            docker rm /openpose-runner
            echo "[$(date +"%T")] removed old docker instance for openpose depth"
            if [ "$docker_exit_code" -ne 0 ]
            then
                echo "[$(date +"%T")] error from docker exit for openpose depth"
                exit "$docker_exit_code"
            fi

            echo "[$(date +"%T")] starting file upload"
            oci os object put \
                --bucket-name $bucket_hdf5 \
                --file "$subj_back1_data_local/full_data-novid.hdf5" \
                --name "$subj_back1/$condition/full_data-novid-poses-depth.hdf5" \
                --force
            rm -rf "$subj_back1_data_local"
    fi

    echo "[$(date +"%T")] done with pose processing"

    rm -rf "$HOME/data/*"
done

echo "--------------------------------------------"
echo "--- Done running pose detection !!!!!!! ----"
echo "--------------------------------------------"
