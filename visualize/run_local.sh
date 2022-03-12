#!/bin/bash
set -o errexit
set -o pipefail

script=$(realpath "$0")
scriptpath=$(dirname "$script")

help_message="
Run visualization code to see what is in the hdf5 files

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
# parse options

pass_through_opts=""

while test $# != 0
do
    case $1 in
        -d|--dir) data=$2; shift;;
        -r|--rebuild-docker) rebuild=true;;
        -h|--help) help=true;;
        *) pass_through_opts="${pass_through_opts} $1"
    esac
    shift
done


if [ "$rebuild" = true ] ; then
    echo 'rebuilding docker image'
    docker build -t video-overlay -f "$scriptpath/../dockerfiles/video_overlay" "$scriptpath/../"
fi

# if help, don't need to map anything...
if [ "$help" = true ] ; then
    echo "$help_message"
    echo "----------------"
    echo "Help from package:"
    docker run \
        --rm \
        -it \
        video-overlay\
        python3 -m visualize.visualize -h
    exit 0
fi

read -ra split_opts <<<"${pass_through_opts}"

docker run \
    --mount type=bind,source="$data",target=/data \
    --rm \
    -it \
    video-overlay\
    python3 -m visualize.visualize --dir "/data/" "${split_opts[@]}"
