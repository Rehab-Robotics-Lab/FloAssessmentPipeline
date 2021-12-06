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
    --config-file "$HOME/.oci/config" \
    --profile 'token-oci-profile' \
    --auth security_token \
    --compartment-id "$flo_compartment" \
    --display-name 'flo-uncompress' \
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
                                                  "blockStorageSizeInGBs": 2000,
                                                  "jobInfrastructureType": "STANDALONE",
                                                  "shapeName": "VM.Standard2.24",
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
    --description 'A job to uncompress the flo data from the raw bucket and put it into the uncompressed bucket' \
    --query 'data.id' \
    --raw-output
)

echo "created new job: $job_id"

echo 'zipping artifacts'
zip -j job.zip "$scriptpath/job.sh" "$scriptpath/../includes/"*
echo 'done zipping'

oci data-science job create-job-artifact \
    --config-file "$HOME/.oci/config" \
    --profile 'token-oci-profile' \
    --auth security_token \
    --job-id "$job_id" \
    --job-artifact-file job.zip \
    --content-disposition 'attachment; filename=job.zip'

echo 'done uploading artifact'

rm job.zip
