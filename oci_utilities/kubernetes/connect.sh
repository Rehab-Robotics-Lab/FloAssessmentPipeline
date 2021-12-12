#!/usr/bin/env bash
set -o errexit
set -o pipefail

## Get OCIDs
echo 'getting OCIDs'
compartment_name='flo'
oke_name='flo-processing'
region='us-ashburn-1'

echo '  getting compartment'
compartment_ocid=$(oci iam compartment list \
    --query "data[? \"name\"=='$compartment_name'] | [0].id" \
    --raw-output)

echo '  getting oke'
oke_ocid=$(oci ce cluster list \
    --compartment-id "$compartment_ocid" \
    --query "data[? \"name\"=='$oke_name'] | [0].id" \
    --raw-output)

echo '  getting oke subnet'
oke_subnet_ocid=$(oci ce cluster get \
    --cluster-id "$oke_ocid" \
    --query 'data."endpoint-config"."subnet-id"' \
    --raw-output)

echo '  getting bastion'
bastion_ocid=$(oci bastion bastion list \
        --compartment-id "$compartment_ocid"\
        --all \
        --query "data[? \"target-subnet-id\"=='$oke_subnet_ocid'] | [0].id" \
        --raw-output)

## Config kubernetes connection
echo 'configuring for connection'
mkdir -p "$HOME/.kube"

oci ce cluster create-kubeconfig \
    --cluster-id "$oke_ocid" \
    --file "$HOME/.kube/config" \
    --region "$region" \
    --token-version 2.0.0 \
    --kube-endpoint PRIVATE_ENDPOINT

export KUBECONFIG=$HOME/.kube/config

## Get IP address and port
cs=$(yq e '.clusters[0].cluster.server' - < "$HOME/.kube/config")
cs=${cs#https://}
ip=${cs%:*}
port=${cs#*:}

## make point to localhost
yq -i e ".clusters[0].cluster.server |= \"https://127.0.0.1:$port\"" ~/.kube/config
## setup clear permissions:
yq -i e '.users[0].user.exec.args += "--profile"' ~/.kube/config
yq -i e ".users[0].user.exec.args += \"$OCI_CLI_PROFILE"\" ~/.kube/config
yq -i e '.users[0].user.exec.args += "--auth"' ~/.kube/config
yq -i e '.users[0].user.exec.args += "security_token"' ~/.kube/config

## Generate new ssh keys
key_name="$HOME/.oci/bastion-keys/bk-$RANDOM"

cleanup(){
    printf "Cleaning up keys\n"
    rm "$key_name"
    rm "$key_name.pub"
    printf "\tDone cleaning up keys\n"

    printf "Shutting down bastion session\n"
    session_resp=$(oci bastion session delete \
        --session-id "$session_id"\
        --wait-for-state "ACCEPTED"\
        --wait-for-state "FAILED"\
        --wait-interval-seconds 5\
        --force \
    )
    printf "\tDone shutting down bastion session\n"
}

trap cleanup EXIT

printf "Generating new ssh-keys\n"
mkdir -p "$HOME/.oci/bastion-keys"
ssh-keygen -f "$key_name" -N "" -q
printf "\tGenerated ssh-keys\n"

printf "Starting new bastion session\n"
session_resp=$(oci bastion session create-port-forwarding \
    --bastion-id "$bastion_ocid" \
    --ssh-public-key-file "$key_name.pub"\
    --key-type 'PUB' \
    --target-private-ip "10.0.0.6" \
    --target-port "$port" \
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
ssh_command=${ssh_command//<privateKey>/"$key_name"}
ssh_command=${ssh_command//<localPort>/"$port"}
ssh_command="$ssh_command -Tv"
eval "$ssh_command" || true
