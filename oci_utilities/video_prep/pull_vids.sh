#!/usr/bin/env bash
set -o errexit
set -o pipefail
export LANG=C.UTF-8
export LC_ALL=C.UTF-8

script="$(realpath "$0")"
scriptpath="$(dirname "$script")"

# shellcheck source=../includes/parse_input_subj_no.sh
source "$scriptpath/../includes/parse_input_subj_no.sh"

mkdir -p "$HOME/Downloads/$subject_padded/3rd-person/"
oci os object bulk-download \
    --config-file "$HOME/.oci/config" \
    --profile 'token-oci-profile' \
    --auth security_token \
    -bn rrl-flo-vids \
    --prefix "$subject_padded/3rd-person/" \
    --download-dir "$HOME/Downloads"

mkdir -p "$HOME/Downloads/$subject_padded/gopro/"
oci os object bulk-download \
    --config-file "$HOME/.oci/config" \
    --profile 'token-oci-profile' \
    --auth security_token \
    -bn rrl-flo-vids \
    --prefix "$subject_padded/gopro/concatenated/transcoded/" \
    --download-dir "$HOME/Downloads"

mkdir -p "$HOME/Downloads/$subject_padded/ros/"
oci os object bulk-download \
    --config-file "$HOME/.oci/config" \
    --profile 'token-oci-profile' \
    --auth security_token \
    -bn rrl-flo-vids \
    --prefix "$subject_padded/ros/transcoded/" \
    --download-dir "$HOME/Downloads"
