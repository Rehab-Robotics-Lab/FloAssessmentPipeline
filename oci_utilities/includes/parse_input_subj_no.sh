#!/usr/bin/env bash

# pass in the subject number
echo "parsing input"
echo "Passed: $1"
re='^[0-9]+$'
if ! [[ $1 =~ $re ]] ; then
    subject_padded="$1"
else
    subject_padded=$(printf '%03d' $(( 10#$1 )))
fi
echo "Processing for subj: $subject_padded"
