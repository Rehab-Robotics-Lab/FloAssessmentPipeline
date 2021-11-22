#!/usr/bin/env bash

# Intended to be sourced from another script
# Will find out the instances ocid and store that in $instance_ocid
# Will figure out the instance compartment id, store it in $instance_compartment_id
# Will check if $PROJECT_COMPARTMENT_OCID is defined (should be defined in data
#   science jobs), and define $compartment as the value of $PROJECT_COMPARTMENT_OCID
#   if it is not defined, then $compartment is set as $instance_compartment_id

# Get some info about where we are running
echo "getting run instance information"
#instance_ocid=$(oci-metadata --get id --value-only)
instance_ocid=$(curl -s -L http://169.254.169.254/opc/v1/instance/ | jq '.id' -r)

echo "instance ocid: $instance_ocid"
availability_domain=$(
    oci compute instance get \
        --instance-id "$instance_ocid" \
        --query 'data."availability-domain"'\
        --raw-output
    )

instance_compartment_id=$(
    oci compute instance get \
        --instance-id "$instance_ocid" \
        --query 'data."compartment-id"'\
        --raw-output
    )
echo "availability domain: $availability_domain"
echo "instance compartment id: $instance_compartment_id"

compartment=''
if [ -z "$PROJECT_COMPARTMENT_OCID" ]
then
    if [ -z "$instance_compartment_id" ]
    then
        echo 'No Compartment ID Found!!!'
    else
        echo "using the instance compartment ocid"
        compartment=$instance_compartment_id
    fi
else
    echo 'using the project ocid'
    # shellcheck disable=SC2034
    compartment=$PROJECT_COMPARTMENT_OCID
fi
