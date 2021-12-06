#!/usr/bin/env bash
set -o errexit
set -o pipefail
export LANG=C.UTF-8
export LC_ALL=C.UTF-8

echo "checking version"
echo "version: $(cat /etc/oracle-release)"

echo "setting permissions"
OCI_CLI_AUTH=resource_principal
export OCI_CLI_AUTH

echo "creating raw data folder"
mkdir -p './data'

echo "downloading files form object storage"
oci os object bulk-download \
    -bn 'rrl-flo-raw' \
    --download-dir './data' \
    --prefix "001" \
    --parallel-operations-count 1000 \
    --overwrite \
    --debug

#echo "downloading file form object storage"
#oci os object get \
#    -bn 'rrl-flo-raw' \
#    --name '001/ros/flo_parameters-2020-11-20-1_.tar' \
#    --file 'file_test'

#echo 'listing all buckets'
#oci os bucket list -c 'ocid1.compartment.oc1..aaaaaaaadznuoh3ntsva2jsj453wwmemd4t2k5rnwniuzkliq7evffxgprua'

#echo 'listing files'
#oci os object list -bn 'rrl-flo-run'
