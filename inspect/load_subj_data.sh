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
echo "Passed: $subject"
re='^[0-9]+$'
if ! [[ $subject =~ $re ]] ; then
   echo "error: Subject number is not a number" >&2; exit 1
fi

subject_padded=$(printf '%03d' "$subject")
echo "Processing for subj: $subject_padded"

src=s3://flo-exp-aim1-data-raw/$subject_padded/
meta=s3://flo-exp-aim1-data-meta/$subject_padded/
#src=/media/mjsobrep/flo-external/008/ # <- this should get pulled in from batch job
# copy from s3 into local storage

# note: subfolders: ros, 3rd-person, gopro

#Get the ros files of interest, note that there are some parameter files that we need to ignore, hence the grep
bag_files=$(aws s3 ls "$src"'ros/'| awk '{print $4}' | grep bag.bz2)

mkdir /data/ros/
mkdir /data/gopro/

aws s3api head-object --bucket flo-exp-aim1-data-meta --key "$subject_padded/meta.yaml" || not_exist=true
if [ $not_exist ]; then
    aws s3 cp s3://flo-exp-aim1-data-meta/template.yaml /data/meta.yaml
else
    aws s3 cp "$meta"'meta.yaml' /data/
fi

prior_pid=-1
for bag_fn in $bag_files
do
    #bring in the bag file from s3
    aws s3 cp "$src"'ros/'"$bag_fn" /data/ros/
    #If we are in the process of uncompressing the prior bag file, wait for that to finish
    if ((prior_pid > 0)); then
        echo "waiting on process $prior_pid"
        wait $prior_pid
    fi
    #In the background, uncompress the bag file
    echo "starting decompression in the background"
    lbzip2 -d "/data/ros/$bag_fn" &
    prior_pid=$!
done

echo "fetching gopro videos"
aws s3 cp --recursive "$src"'gopro' /data/gopro/

echo "-------------------------------"
echo "--- Done fetching data !!!!----"
echo "-------------------------------"
