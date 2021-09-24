#!/bin/bash
set -o errexit
set -o pipefail

params=""
# parse options
while (( "$#" )); do
  case "$1" in
    -d|--dir)
      if [ -n "$2" ] && [ ${2:0:1} != "-" ]; then
        dir=$2
        shift 2
      else
        echo "Error: Argument for $1 is missing" >&2
        exit 1
      fi
      ;;
    *) # preserve positional arguments
      params="$params $1"
      shift
      ;;
  esac
done

docker run \
    --mount type=bind,source="$dir",target=/data \
    -it \
    --rm \
    --name=bag2video \
    bag2video "$params"
