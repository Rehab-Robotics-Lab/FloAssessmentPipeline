#!/usr/bin/env bash
set -o errexit
set -o pipefail

BUCKET_NAME='rrl-flo-raw'

while getopts :p:t: flag
do
    case "${flag}" in
        t) src_dir=${OPTARG};;
        p) prefix=${OPTARG};;
        :) echo 'missing argument' >&2; exit 1;;
        \?) echo 'invalid option' >&2; exit 1
    esac
done

prefix="${prefix%/}/"

refresh_fun(){
    echo "token refresh background job started (PID=$BASHPID)"
    while :
    do
        echo 'background job refreshing token'
        sleep $((  60*30  )) # 60 sec * 30 minutes
        oci session refresh --profile 'rrl-oci-profile-data-upload'
    done
    echo 'exiting background job'
}

oci session authenticate \
    --region us-ashburn-1 \
    --tenancy-name 'upennrehabrobotics' \
    --profile-name 'rrl-oci-profile-data-upload'

refresh_fun &
background_pid=$!
echo "launched background token maintenance (PID=$background_pid)"

cleanup(){
    echo ''
    printf "Cleaning up session\n"
    kill "$background_pid" || true
    oci session terminate \
    --config-file "$HOME/.oci/config" \
    --profile 'rrl-oci-profile-data-upload' \
    --auth security_token \
    echo 'done cleaning up'
}

trap cleanup EXIT

oci os object sync \
    --config-file "$HOME/.oci/config" \
    --profile 'rrl-oci-profile-data-upload' \
    --auth security_token \
    --bucket-name $BUCKET_NAME \
    --dry-run \
    --src-dir "$src_dir" \
    --prefix "$prefix"
echo "will upload with prefix: $prefix"

while :
do
    read  -r -n 1 -p "would you like to proceed with upload? (y/n)" proceed
    echo ''

    if [ "$proceed" = "n" ]; then
        echo 'you said no'
       exit 0
    elif [ "$proceed" = "y" ];then
        echo 'you said yes'
        break
    fi
    echo 'you need to say yes: y or no: n!!'
done

echo 'beginning upload'
oci os object sync \
    --config-file "$HOME/.oci/config" \
    --profile 'rrl-oci-profile-data-upload' \
    --auth security_token \
    --bucket-name $BUCKET_NAME \
    --src-dir "$src_dir" \
    --prefix "$prefix"

echo 'upload complete'
