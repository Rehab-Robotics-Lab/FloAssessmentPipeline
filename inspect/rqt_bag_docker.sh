#!/bin/bash
set -o errexit
set -o pipefail

script="$(realpath "$0")"
scriptpath="$(dirname "$script")"

prior=$(pwd)
cleanup(){
    cd "$prior"
}
trap cleanup EXIT
cd "$scriptpath"

RQT_USER=$(id -u "$USER"):$(id -g "$USER")
export RQT_USER
docker-compose up
