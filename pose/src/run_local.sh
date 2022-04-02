#!/bin/bash
set -o errexit
set -o pipefail

script=$(realpath "$0")
scriptpath=$(dirname "$script")

help_message="
Extract poses from video in hdf files.

Args passed through to underlying script. Everything
runs in docker. If the docker image does not exist,
no additional help will print.

Additional Args for wrapper:
 -r|--rebuild-docker: rebuild the docker file that this
                      uses if this flag is present
 -h|--help: print help

"

rebuild=false
help=false

pass_through_opts=""

while test $# != 0
do
    case $1 in
        -d|--dir) data=$2; shift;;
        -r|--rebuild-docker) rebuild=true;;
        -h|--help) help=true;;
        -a|--algorithm) algorithm=$2;;& # ;;& will test remaining cases. in this case we ant to pass this through in adition to taking a look
        *) pass_through_opts="${pass_through_opts} $1"
    esac
    shift
done

if [ "$rebuild" = true ] ; then
    echo 'rebuilding docker images'
    #docker build -t openpose -f "$scriptpath/../../dockerfiles/openpose" "$scriptpath/../../"
    docker build -t mediapipe -f "$scriptpath/../../dockerfiles/mediapipe" "$scriptpath/../../"
fi

# if help, don't need to map anything...
if [ "$help" = true ] ; then
    echo "$help_message"
    echo "----------------"
    echo "Help from package:"
    docker run \
        --rm \
        -it \
        mediapipe\
        python3 -m pose.src.process_hdf5 -h
    exit 0
fi

read -ra split_opts <<<"${pass_through_opts}"

read -ra split_opts <<<"${pass_through_opts}"

if [[ "$algorithm" == "openpose:"* ]]
then
    docker run \
        --mount type=bind,source="$data",target=/data \
        --rm \
        -it \
        openpose\
        python3 -m pose.src.process_hdf5 -d "/data/" "${split_opts[@]}"
elif [[ "$algorithm" == "mp-hands" ]]
then
    docker run \
        --mount type=bind,source="$data",target=/data \
        --rm \
        -it \
        mediapipe\
        python3 -m pose.src.process_hdf5 -d "/data/" "${split_opts[@]}"
fi
