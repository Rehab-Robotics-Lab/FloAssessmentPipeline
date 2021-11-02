#!/usr/bin/env bash
set -o errexit
set -o pipefail

## Authenticate an OCI session and keep it alive. As long as this is running
## (up to 24 hours), your token should work.


oci session authenticate --region us-ashburn-1 --tenancy-name 'upennrehabrobotics' --profile-name 'token-oci-profile'
while :
do
    sleep $((  60*30  )) # 60 sec * 30 minutes
    oci session refresh --profile 'token-oci-profile'
done
