#!/bin/bash
set -o errexit
set -o pipefail

script="$(realpath "$0")"
scriptpath="$(dirname "$script")"

mkdir -p "$HOME/data"
rm -rf "$HOME/data/*"
oci os object bulk-download \
    -bn rrl-flo-hdf5 \
    --include '*novid.hdf5' \
    --download-dir "$HOME/data"

python3 "$scriptpath/../../get_transforms/run.py" -t "$HOME/data"

oci os object put \
    -bn 'rrl-flo-transforms' \
    --file "$HOME/data/transforms.json" \
    --force \
    --name 'transforms.json'
