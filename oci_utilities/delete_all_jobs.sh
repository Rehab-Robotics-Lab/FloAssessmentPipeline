#!/usr/bin/env bash
set -o errexit
set -o pipefail


flo_compartment='ocid1.compartment.oc1..aaaaaaaadznuoh3ntsva2jsj453wwmemd4t2k5rnwniuzkliq7evffxgprua'

# shellcheck disable=SC2016
# should not expand query string
for job in $(oci data-science job list \
    --config-file "$HOME/.oci/config" \
    --profile 'token-oci-profile' \
    --auth security_token \
    --compartment-id "$flo_compartment" \
    --all \
    --query 'data[? "lifecycle-state" == `CREATING` || "lifecycle-state" == `ACTIVE`].id' \
    --raw-output | \
    jq -r '.[]')
do
    echo "working on job: $job"
    for job_run in $(oci data-science job-run list \
        --config-file "$HOME/.oci/config" \
        --profile 'token-oci-profile' \
        --auth security_token \
        --compartment-id "$flo_compartment" \
        --all \
        --job-id "$job" \
        --query 'data[].id' \
        --raw-output | \
        jq -r '.[]'
        )
    do
        echo "    working on job run: $job_run"
        job_state=$(oci data-science job-run get \
            --config-file "$HOME/.oci/config" \
            --profile 'token-oci-profile' \
            --auth security_token \
            --job-run-id "$job_run" \
            --query 'data."lifecycle-state"' \
            --raw-output)
        if [ "$job_state" = 'IN_PROGRESS' ]
        then
            echo "      trying to cancel job"
            oci data-science job-run cancel \
                --config-file "$HOME/.oci/config" \
                --profile 'token-oci-profile' \
                --auth security_token \
                --job-run-id "$job_run" \
                >/dev/null
            echo '*** You will probably need to run this again once jobs have finished being cancelled ***'
        fi
        oci data-science job-run delete \
            --config-file "$HOME/.oci/config" \
            --profile 'token-oci-profile' \
            --auth security_token \
            --job-run-id "$job_run" \
            --force \
            --wait-for-state 'SUCCEEDED' \
            --wait-for-state 'DELETED' \
            --wait-for-state 'FAILED' \
            >/dev/null && true
        echo "      done with job run"
    done
    echo "  done with all job runs"
    oci data-science job delete \
        --config-file "$HOME/.oci/config" \
        --profile 'token-oci-profile' \
        --auth security_token \
        --job-id "$job" \
        --force \
        --wait-for-state SUCCEEDED \
        --wait-for-state FAILED \
        --wait-interval-seconds 1 \
        >/dev/null
    echo "  done with job"
done
echo "done with all"
