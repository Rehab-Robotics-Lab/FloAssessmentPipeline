#!/bin/bash
set -o errexit
set -o pipefail

# parse options
while getopts :s: flag
do
    case "${flag}" in
        s) subject=${OPTARG};;
        :) echo 'missing argument' >&2; exit 1;;
        \?) echo 'invalid option' >&2; exit 1
    esac
done

# check subject number
re='^[0-9]+$'
if ! [[ $subject =~ $re ]] ; then
   echo "error: Subject number is not a number" >&2; exit 1
fi

subject_padded=$(printf '%03d' "$subject")
echo "Processing for subj: $subject_padded"

src=flo-exp-aim1-data-raw/$subject_padded/
out=flo-exp-aim1-hdf5/$subject_padded/
#src=/media/mjsobrep/flo-external/008/ # <- this should get pulled in from batch job
# copy from s3 into local storage

# note: subfolders: ros, 3rd-person, gopro

bag_files=$(aws s3 ls "$src"'ros/'| awk '{print $4}' | grep bag.bz2)

aws s3 cp "$src"'meta.yaml' .

prior_pid=-1
for bag_fn in $bag_files
do
    aws s3 cp "$src"'ros/'"$bag_fn" .
    if ((prior_pid > 0)); then
        wait $prior_pid
    fi
    lbzip2 -d "$bag_fn" && put into hdf5 && rm ${$bag_fn:0:-4} &
    prior_pid=$!
done

aws s3 cp ./*.hdf5 "$out"
