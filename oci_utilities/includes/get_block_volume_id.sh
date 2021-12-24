#!/usr/bin/env bash

# Should be sourced from another script
# Gets the ID for a block volume, automatically selecting a compartment
#   from available options, if not already set in $compartment, will put
#   it there
# Stores the block volume id in $volume_id
# Finds block volume by looking at $subject_padded, which must be defined
#   prior to running this script. Expects block volumes to be named as:
#   `flo_data/$subject_padded`

echo "getting block volume id"

if [ -z "$compartment" ]
then
    if [ -z "$PROJECT_COMPARTMENT_OCID" ]
    then
        # shellcheck disable=SC2154
        # we are checking if this variable exists...
        if [ -z "$instance_compartment_id" ]
        then
            echo 'No Compartment ID Found!!!'
        else
            echo "using the instance compartment ocid"
            compartment=$instance_compartment_id
        fi
    else
        echo 'using the project ocid'
        compartment=$PROJECT_COMPARTMENT_OCID
    fi
fi

# shellcheck disable=SC2154,SC2016
# Dont want to have shell expend this stuff (SC2016)
volume_id=$(
    oci bv volume list \
        --compartment-id "$compartment" \
        --display-name "flo_data/$subject_padded" \
        --query 'data[? "lifecycle-state" == `"AVAILABLE"`] | [0].id' \
        --raw-output
)
echo "block storage volume ocid: $volume_id"
