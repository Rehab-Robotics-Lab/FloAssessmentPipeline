#!/usr/bin/env bash
set -o errexit
set -o pipefail

script="$(realpath "$0")"
scriptpath="$(dirname "$script")"

oci os object bulk-upload \
    -bn 'rrl-flo-run' \
    --src-dir "$scriptpath/" \
    --prefix 'video_prep/' \
    --overwrite \
    --config-file "$HOME/.oci/config"\
    --profile 'token-oci-profile'\
    --auth security_token

oci os object bulk-upload \
    -bn 'rrl-flo-run' \
    --src-dir "$scriptpath/../includes/" \
    --prefix 'video_prep/' \
    --overwrite \
    --config-file "$HOME/.oci/config"\
    --profile 'token-oci-profile'\
    --auth security_token

oci os object bulk-upload \
    -bn 'rrl-flo-run' \
    --src-dir "$scriptpath/../../bag2video/" \
    --prefix 'video_prep/' \
    --overwrite \
    --config-file "$HOME/.oci/config"\
    --profile 'token-oci-profile'\
    --auth security_token

oci os object bulk-upload \
    -bn 'rrl-flo-run' \
    --src-dir "$scriptpath/../../prep_code_vids/" \
    --prefix 'video_prep/' \
    --overwrite \
    --config-file "$HOME/.oci/config"\
    --profile 'token-oci-profile'\
    --auth security_token
