#!/usr/bin/env bash
set -o errexit
set -o pipefail
export LANG=C.UTF-8
export LC_ALL=C.UTF-8

script="$(realpath "$0")"
scriptpath="$(dirname "$script")"


# shellcheck source=../includes/check_version.sh
source "$scriptpath/check_version.sh"

#pip install oci-cli --upgrade

if [ -z "$(lbzip2 --version)" ]
then
    # shellcheck source=../includes/add_extended_repos.sh
    source "$scriptpath/add_extended_repos.sh"
    echo "installing lbzip2"
    sudo dnf install -y -q lbzip2
fi

# shellcheck source=../includes/set_permissions_job.sh
source "$scriptpath/set_permissions_job.sh"


# shellcheck source=../includes/parse_input_subj_no.sh
source "$scriptpath/parse_input_subj_no.sh"


# copy and decompress data
echo "creating raw data folder"
mkdir -p './data'
echo "downloading files from object storage"
download_result=$(oci os object bulk-download \
    -bn 'rrl-flo-raw' \
    --download-dir './data' \
    --prefix "$subject_padded" \
    --parallel-operations-count 500 \
    --overwrite)
echo "download result: $download_result"

#errors=1
#while (( errors > 0 ))
#do
#    echo "Starting sync"
#    errors=$(oci os object sync \
#        -bn 'rrl-flo-raw' \
#        --dest-dir './data' \
#        --prefix "$subject_padded" \
#        --parallel-operations-count 500 \
#        | jq '."download-failures" | length')
#    echo "sync finished with $errors failures to download"
#done

#echo "removing subject number prefix from directories"
##rsync -a "/mnt/subj-data/raw/$subject_padded/"* /mnt/subj-data/raw
#cp -rlf "./subj-data/raw/$subject_padded/"* ./subj-data/raw
#rm "./subj-data/raw/$subject_padded" -r

echo "uncompressing files"
find ./data -name '*.bz2' -exec lbzip2 -d -f -n 48 {} \;
echo "untarring files"
find ./data -name '*.tar' -exec bash -c 'tar -xf "$1" --directory "$(dirname "$1")"' shell {} \;

# push back to os
echo 'uploading uncompressed files'
upload_result=$(oci os object bulk-upload \
    -bn 'rrl-flo-uncompressed' \
    --src-dir ./data \
    --overwrite \
    --parallel-upload-count 500)
echo "upload result: $upload_result"

#errors=1
#while (( errors > 0 ))
#do
#    echo "starting sync"
#    errors=$(oci os object sync \
#        -bn 'rrl-flo-uncompressed' \
#        --src-dir './data' \
#        --parallel-operations-count 500 \
#        | jq '."upload-failures" | length')
#    echo "sync finished with $errors failures to upload"
#done

echo "job complete"
