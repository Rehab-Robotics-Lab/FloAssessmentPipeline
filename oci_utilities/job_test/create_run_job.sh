#!/usr/bin/env bash
set -o errexit
set -o pipefail

script="$(realpath "$0")"
scriptpath="$(dirname "$script")"

flo_compartment='ocid1.compartment.oc1..aaaaaaaadznuoh3ntsva2jsj453wwmemd4t2k5rnwniuzkliq7evffxgprua'
log_group='ocid1.loggroup.oc1.iad.amaaaaaachblvpyabmawfuu3pxxyr4ezwdgqfyveaxmzfcgi3cp3hhhvnkwq'
subnet='ocid1.subnet.oc1.iad.aaaaaaaalvfmp226glz3pt4wbxn5rfnwpsnwriyjrrdmg3xocjhjmb6xd3ma'
project='ocid1.datascienceproject.oc1.iad.amaaaaaachblvpya4rjbrewjek7rrwor7qhiigpzkmptfnddjkexxcui4mca'

job_id=$(oci data-science job create \
    --compartment-id "$flo_compartment" \
    --display-name "$1" \
    --project-id "$project" \
    --configuration-details "$(jq --null-input \
                                  '{
                                  "jobType": "DEFAULT",
                                  "maximumRuntimeInMinutes": 240
                                  }'
                              )" \
    --infrastructure-configuration-details "$(jq --null-input \
                                                 --arg subnet $subnet \
                                                 '{
                                                  "blockStorageSizeInGBs": 200,
                                                  "jobInfrastructureType": "STANDALONE",
                                                  "shapeName": "VM.Standard2.4",
                                                  "subnetId": $subnet
                                                  }'
                                             )" \
    --log-configuration-details "$(jq --null-input \
                                      --arg logGroupId $log_group \
                                          '{
                                          "enableAutoLogCreation": true,
                                          "enableLogging": true,
                                          "logGroupId": $logGroupId
                                        }'
                                    )"\
    --description 'job testing' \
    --query 'data.id' \
    --raw-output
)

echo "created new job: $job_id"


oci data-science job create-job-artifact \
    --job-id "$job_id" \
    --job-artifact-file "$scriptpath"/min.sh \
    --content-disposition 'attachment; filename=min.sh'

echo 'done uploading artifact'

echo 'waiting for job to be ready'

lifecycle_state="unknown"
until [ "$lifecycle_state" = 'ACTIVE' ]
do
    lifecycle_state=$(oci data-science job get \
        --job-id "$job_id" \
        --query 'data."lifecycle-state"' \
        --raw-output)
    echo "Lifecycle State: $lifecycle_state"
    sleep 1
done

echo 'running job'

job_run=$(oci data-science job-run create \
    --compartment-id "$flo_compartment" \
    --job-id "$job_id" \
    --project-id "$project" \
    --display-name "test" \
    --configuration-override-details "$(jq --null-input \
                                           '{"commandLineArguments": "hi",
                                             "jobType":"DEFAULT",
                                             "environmentVariables":{
                                                "JOB_RUN_ENTRYPOINT": "min.sh"
                                         }}'
                                       )" \
    --query 'data.id' \
    --raw-output
)

echo "view job run at:    https://console.us-ashburn-1.oraclecloud.com/data-science/job-runs/$job_run"
