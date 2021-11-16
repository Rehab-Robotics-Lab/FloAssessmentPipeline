#!/usr/bin/env bash
set -o errexit
set -o pipefail

script="$(realpath "$0")"
scriptpath="$(dirname "$script")"

oci os object put \
    -bn 'rrl-flo-run' \
    --file "$scriptpath/run.sh" \
    --name 'block_storage_setup/run.sh' \
    --force \
    --config-file "$HOME/.oci/config"\
    --profile 'token-oci-profile'\
    --auth security_token

oci os object bulk-upload \
    -bn 'rrl-flo-run' \
    --src-dir "$scriptpath/../includes/" \
    --prefix 'block_storage_setup/' \
    --overwrite \
    --config-file "$HOME/.oci/config"\
    --profile 'token-oci-profile'\
    --auth security_token
