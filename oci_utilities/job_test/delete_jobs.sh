#!/usr/bin/env bash
set -o errexit
set -o pipefail

script="$(realpath "$0")"
scriptpath="$(dirname "$script")"

flo_compartment='ocid1.compartment.oc1..aaaaaaaadznuoh3ntsva2jsj453wwmemd4t2k5rnwniuzkliq7evffxgprua'
log_group='ocid1.loggroup.oc1.iad.amaaaaaachblvpyabmawfuu3pxxyr4ezwdgqfyveaxmzfcgi3cp3hhhvnkwq'
subnet='ocid1.subnet.oc1.iad.aaaaaaaalvfmp226glz3pt4wbxn5rfnwpsnwriyjrrdmg3xocjhjmb6xd3ma'
project='ocid1.datascienceproject.oc1.iad.amaaaaaachblvpya4rjbrewjek7rrwor7qhiigpzkmptfnddjkexxcui4mca'


for job in $(oci data-science job list \
    --config-file "$HOME/.oci/config" \
    --profile 'token-oci-profile' \
    --auth security_token \
    --compartment-id "$flo_compartment" \
    --display-name 'flo-uncompress' \
    --all \
    --query 'data[? "lifecycle-state" == `CREATING` || "lifecycle-state" == `ACTIVE`].id' \
    --raw-output | \
    jq -r '.[]')
do
    oci data-science job delete \
        --config-file "$HOME/.oci/config" \
        --profile 'token-oci-profile' \
        --auth security_token \
        --job-id "$job" \
        --force \
        --wait-for-state ACCEPTED \
        --wait-for-state FAILED \
        --wait-interval-seconds 1
done
