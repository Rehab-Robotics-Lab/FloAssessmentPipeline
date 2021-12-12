#!/bin/bash

script="$(realpath "$0")"
scriptpath="$(dirname "$script")"

echo 'syncing data out'

readarray -t ignored_files < <(git ls-files --others --ignored --exclude-standard)

echo "excluding:"
echo "${ignored_files[@]}"

for elem in "${ignored_files[@]}"; do
    exclude_opts+=( --exclude "$elem" )
done


oci os object sync \
    -bn 'rrl-flo-run' \
    --delete \
    --exclude '.git*' \
    "${exclude_opts[@]}" \
    --src-dir "$scriptpath/../"

echo 'done syncing data'
