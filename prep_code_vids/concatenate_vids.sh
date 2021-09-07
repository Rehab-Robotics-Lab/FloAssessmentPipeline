#!/bin/bash
set -o errexit
set -o pipefail

# parse options
while getopts :t:o: flag
do
    case "${flag}" in
        t) target=${OPTARG};;
        o) output=${OPTARG};;
        :) echo 'missing argument' >&2; exit 1;;
        \?) echo 'invalid option' >&2; exit 1
    esac
done

target=${target%/}
output=${output%/}
mkdir -p "$output"
echo "searching in: $target/"
readarray -t files <<<"$(ls "$target"/GH01*MP4)"
for fn in "${files[@]}"
do
    fn_clean=${fn#"$target/"}
    echo "found first file in seq: $fn_clean"
    vid_num=${fn_clean#"GH01"}
    vid_num=${vid_num%".MP4"}
    echo "vid num: $vid_num"
    keep_going=true
    highest_found=1
    while $keep_going
    do
        printf -v next_fn "$target/GH%02d$vid_num.MP4" $highest_found
        echo "looking for $next_fn"
        if test -f "$next_fn"
        then
            echo "found next file: $next_fn"
            ((highest_found++))
        else
            echo "no more files in this sequence"
            keep_going=false
        fi
    done
    ((highest_found--))
    echo "highest found: $highest_found"
    fo=$output/$vid_num.MP4
    echo "concatenating to: $fo"
    ffmpeg -f concat -safe 0 -i <(for f in $(seq 1 $highest_found); do printf "file '$target/GH%02d$vid_num.MP4'\n" "$f"; done) -c copy "$fo"
    echo "-------------"
done
