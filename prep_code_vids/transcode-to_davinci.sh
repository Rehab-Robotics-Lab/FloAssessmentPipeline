#!/bin/bash
set -o errexit
set -o pipefail

# parse options
while getopts :t: flag
do
    case "${flag}" in
        t) target=${OPTARG};;
        :) echo 'missing argument' >&2; exit 1;;
        \?) echo 'invalid option' >&2; exit 1
    esac
done

target=${target%/}
echo "searching in: $target/"
output="$target/transcoded"
mkdir -p "$output"

echo "putting videos in $output"

readarray -t files < <(find "$target" -type f -iname '*MP4')
for fn in "${files[@]}"
do
    fn_base=${fn#"$target/"}
    fn_out="$output/${fn_base%.*}.mov"
    echo "working on: $fn;; outputing to: $fn_out"

    ffmpeg -i "$fn" -vcodec mjpeg -q:v 2 -acodec pcm_s16be -q:a 0 -f mov "$fn_out"
    echo "-------------"
done
