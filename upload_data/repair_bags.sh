#!/bin/bash
set -o errexit
set -o pipefail
shopt -s globstar

# parse options
while getopts :t:h flag
do
    case "${flag}" in
        t) target=${OPTARG};;
        h) echo 'Will recursively find all active bags in a directory, index and fix them. Requires one argument: -t <parent dir>'; exit 0;;
        :) echo 'missing argument (t)arget' >&2; exit 1;;
        \?) echo 'invalid option' >&2; exit 1
    esac
done

echo "searching in dir: $target"

find "$target" -wholename "**/*\.bag\.active" -print0 | while read -r -d $'\0' file
do
    echo "working on: $file this may take a few minutes"
    clean_file=${file#"$target"}
    clean_file=${clean_file#/}
    docker run --volume "$target":/target osrf/ros:melodic-desktop-full rosbag reindex -f "/target/$clean_file" && \
        docker run --volume "$target":/target osrf/ros:melodic-desktop-full rosbag fix --force "/target/$clean_file" "/target/${clean_file%.active}" && \
        rm "$target/$clean_file" && \
        rm "$target/${clean_file%.active}.orig.active"
done
