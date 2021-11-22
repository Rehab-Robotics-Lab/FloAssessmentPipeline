#!/usr/bin/env bash
set -o errexit
set -o pipefail

script="$(realpath "$0")"
scriptpath="$(dirname "$script")"

flo_compartment='ocid1.compartment.oc1..aaaaaaaadznuoh3ntsva2jsj453wwmemd4t2k5rnwniuzkliq7evffxgprua'
project='ocid1.datascienceproject.oc1.iad.amaaaaaachblvpya4rjbrewjek7rrwor7qhiigpzkmptfnddjkexxcui4mca'

if [ "$1" == 'all' ]
then
    # shellcheck source=../includes/get_all_subj.sh
    subjects=$(oci os object list \
        --config-file "$HOME/.oci/config" \
        --profile 'token-oci-profile' \
        --auth security_token \
        -bn 'rrl-flo-uncompressed' \
        --all \
        --query 'data[].name' \
        | jq 'map(split("/")[0]) | unique | .[]' -r)
else
    # shellcheck source=../includes/parse_input_subj_no.sh
    source "$scriptpath/../includes/parse_input_subj_no.sh"
    subjects="$subject_padded"
fi

job=$(oci data-science job list \
    --config-file "$HOME/.oci/config" \
    --profile 'token-oci-profile' \
    --auth security_token \
    --compartment-id "$flo_compartment" \
    --display-name 'flo-video-prep' \
    --all \
    --query 'data[? "lifecycle-state" == `ACTIVE`] | [0].id' \
    --raw-output)

echo "posting runs to job: $job"

while read -r subject
do
    job_run=$(oci data-science job-run create \
        --config-file "$HOME/.oci/config" \
        --profile 'token-oci-profile' \
        --auth security_token \
        --compartment-id "$flo_compartment" \
        --job-id "$job" \
        --project-id "$project" \
        --configuration-override-details "$(jq --null-input \
                                               --arg subject "$subject" \
                                               '{"commandLineArguments": $subject,
                                                 "jobType":"DEFAULT",
                                                 "environmentVariables":{
                                                    "JOB_RUN_ENTRYPOINT": "job.sh"
                                             }}'
                                           )" \
        --display-name "flo-video-prep-$subject" \
        --query 'data.id' \
        --raw-output
    )

    echo "started job for subject $subject on job run $job_run"
done <<<"$subjects"
