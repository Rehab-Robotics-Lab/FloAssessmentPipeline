#!/bin/bash
set -o errexit
set -o pipefail
shopt -s globstar

# parse options
while getopts :t:h flag
do
    case "${flag}" in
        t) target=${OPTARG};;
        h) echo 'Will recursively find all meta yaml files in a directory and pull them together into a tar file. Requires one argument: -t <parent dir>'; exit 0;;
        :) echo 'missing argument (t)arget' >&2; exit 1;;
        \?) echo 'invalid option' >&2; exit 1
    esac
done

echo "searching in dir: $target"

find "$target" -wholename "**/*\.yaml" -print0 | while read -r -d $'\0' file
do
    #echo "working on: $file"
    base=${file%flo_parameters*}
    base=${base%/}
    datetime=${file#*flo_parameters-}
    date=${datetime:0:10}
    echo "datetime:  $datetime"
    echo "Working in: $base"
    cd "$base"
    echo "Working on date: $date"
    tar -rvf "flo_parameters-$date-_.tar" "flo_parameters-$datetime" && rm "flo_parameters-$datetime"
    cd -
done
