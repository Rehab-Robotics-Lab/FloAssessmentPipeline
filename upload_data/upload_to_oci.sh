#!/usr/bin/env bash
set -o errexit
set -o pipefail

BUCKET_NAME='sandbox'

while getopts :s: flag
do
    case "${flag}" in
        s) src_dir=${OPTARG};;
        :) echo 'missing argument' >&2; exit 1;;
        \?) echo 'invalid option' >&2; exit 1
    esac
done

refresh_fun(){
    while :
    do
        sleep $((  60*30  )) # 60 sec * 30 minutes
        oci session refresh --profile 'rrl-oci-profile'
    done

}

oci session authenticate --region us-ashburn-1 --tenancy-name 'upennrehabrobotics' --profile-name 'rrl-oci-profile'
refresh_fun &
background_pid=$!

oci os object sync --bucket-name $BUCKET_NAME --dry-run --src-dir "$src_dir"

while :
do
    read  -r -n 1 -p "would you like to proceed with upload? (y/n)" proceed

    if [ "$proceed" = "n" ]; then
       kill $background_pid
       exit 0
    elif [ "$proceed" = "y" ];then
        break
    fi
done

oci os object sync --bucket-name $BUCKET_NAME --src-dir "$src_dir"

kill $background_pid
echo 'done cleaning up'
