#!/usr/bin/env bash
set -o errexit
set -o pipefail


flo_compartment='ocid1.compartment.oc1..aaaaaaaadznuoh3ntsva2jsj453wwmemd4t2k5rnwniuzkliq7evffxgprua'


# shellcheck disable=SC2016
# should not expand query string
for job in $(oci data-science job list \
    --compartment-id "$flo_compartment" \
    --display-name 'flo-uncompress' \
    --all \
    --query 'data[? "lifecycle-state" == `CREATING` || "lifecycle-state" == `ACTIVE`].id' \
    --raw-output | \
    jq -r '.[]')
do
    oci data-science job delete \
        --job-id "$job" \
        --force \
        --wait-for-state ACCEPTED \
        --wait-for-state FAILED \
        --wait-interval-seconds 1
done
