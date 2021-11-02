#!/usr/bin/env bash
set -o errexit
set -o pipefail

instance_id=$1

instance_info=$(
    oci compute instance get \
        --config-file "$HOME/.oci/config"\
        --profile "token-oci-profile"\
        --auth security_token\
        --instance-id "$instance_id"
)
instance_name=$(echo "$instance_info" | jq '.data."display-name"')
instance_compartment=$(echo "$instance_info" | jq '.data."compartment-id"' -r)

printf "Attempting to connect to %s\n" "$instance_name"

instance_state=$(echo "$instance_info" | jq '.data."lifecycle-state"' -r)
if [ "$instance_state" != 'RUNNING' ]; then
    printf 'The target is not running\n'
    exit 1
fi

plugin_info=$(
    oci instance-agent plugin list\
        --config-file "$HOME/.oci/config"\
        --profile "token-oci-profile"\
        --auth security_token\
        --compartment-id "$instance_compartment"\
        --instanceagent-id "$instance_id"\
        --all
)

bastion_plugin_status=$(echo "$plugin_info" | jq '.data[] | select(.name=="Bastion") | .status' -r)
if [ "$bastion_plugin_status" != 'RUNNING' ]; then
    printf 'The target instance does not have the bastion service enabled\n'
    exit 1
fi


subnets=$(
    oci compute instance list-vnics \
        --config-file "$HOME/.oci/config"\
        --profile "token-oci-profile"\
        --auth security_token\
        --instance-id "$instance_id"
)

bastions=$(
    oci bastion bastion list \
        --config-file "$HOME/.oci/config"\
        --profile "token-oci-profile"\
        --auth security_token\
        --compartment-id "$instance_compartment"\
        --all
)

for subnet_id in $(echo "$subnets"|jq '.data[]."subnet-id"' -r);
do
    bastion_id=$(echo "$bastions" | jq ".data[] | select(.\"target-subnet-id\"==\"$subnet_id\") | .id" -r)
    [[ -n "$bastion_id" ]] && break
done

if [ -z "$bastion_id" ]
then
    printf "No bastions found for this instance\nexiting\n"
    exit 1
fi

printf "Generating new ssh-keys\n"
mkdir -p "$HOME/.oci/bastion-keys"
key_name="$HOME/.oci/bastion-keys/bk-$RANDOM"
ssh-keygen -f "$key_name" -N "" -q
printf "\tGenerated ssh-keys\n"

printf "Starting new bastion session\n"
session_resp=$(oci bastion session create-managed-ssh \
    --config-file "$HOME/.oci/config"\
    --profile "token-oci-profile"\
    --auth security_token\
    --bastion-id "$bastion_id" \
    --target-resource-id "$instance_id"\
    --ssh-public-key-file "$key_name.pub"\
    --target-os-username "opc"\
    --session-ttl "$((60*60*3))"\
    --wait-for-state "SUCCEEDED"\
    --wait-for-state "FAILED"\
    --wait-interval-seconds 5\
)
printf "\tStarted bastion session\n"

printf "Getting new session info\n"
session_id=$(echo "$session_resp" | jq '.data.resources' | jq '.[0].identifier' -r)
session_data=$(oci bastion session get\
    --session-id "$session_id"\
    --config-file "$HOME/.oci/config"\
    --profile "token-oci-profile"\
    --auth security_token\
    --raw-output\
)
printf "\tReceived new session info\n"

printf "Connecting to remote host\n"
ssh_command=$(echo "$session_data" |\
    tr '\r\n' ' ' |\
    jq '.data."ssh-metadata".command' -r\
)
ssh_command="${ssh_command//<privateKey>/$key_name}"
eval "$ssh_command"

printf "SSH session ended\n"

printf "Cleaning up keys\n"
rm "$key_name"
rm "$key_name.pub"
printf "\tDone cleaning up keys\n"

printf "Shutting down bastion session\n"
session_resp=$(oci bastion session delete \
    --config-file "$HOME/.oci/config"\
    --profile "token-oci-profile"\
    --auth security_token\
    --session-id "$session_id"\
    --wait-for-state "ACCEPTED"\
    --wait-for-state "FAILED"\
    --wait-interval-seconds 5\
    --force \
)
printf "\tDone shutting down bastion session\n"
exit 0
