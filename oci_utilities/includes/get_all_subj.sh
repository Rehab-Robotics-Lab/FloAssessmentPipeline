#!/usr/bin/env bash

subjects=$(oci os object list \
    --config-file "$HOME/.oci/config" \
    --profile 'token-oci-profile' \
    --auth security_token \
    -bn 'rrl-flo-raw' \
    --all \
    --query 'data[].name' \
    | jq 'map(split("/")[0]) | unique | .[]' -r)
