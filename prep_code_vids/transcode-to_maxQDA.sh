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

readarray -t files <<<"$(find "$target" -type f -iname '*mov')"
for fn in "${files[@]}"
do
    fn_base=${fn#"$target/"}
    fn_out="$output/${fn_base%.*}.mp4"
    echo "working on: $fn;; outputing to: $fn_out"
    #https://www.maxqda.com/faq/media-files
    #MAXQDA supports the the following formats as standard:
        #Audio	Windows: MP3, WAV, WMA, AAC, M4A
        #Mac: MP3, WAV, AAC, CAF, M4A
        #Video	MP4, MOV, MPG, AVI, M4V, 3GP, 3GGP
        #Windows: additionally WMV
        #For video, a MP4 file with the video codec H.264 / AVC is recommended.

# -codec:v libx265: codec to encode
# -crf 18 selects the constant rate factor to be high quality
# -preset veryfast tells the algorithm to run fast, at the cost of a bigger file
# -codec c:a aac : sets the audio codec
# -vf format=yuv420p:  chooses YUV 4:2:0 chroma-subsampling which is recommended for H.264 compatibility.
# - movflags +faststart: moves headers to the front of lile to enable faster play on partial download
    ffmpeg -i "$fn" -codec:v libx264 -crf 18 -preset veryfast -codec:a aac -vf format=yuv420p -movflags +faststart "$fn_out"
    echo "-------------"
done
