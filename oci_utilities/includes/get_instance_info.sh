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
    compartment=$PROJECT_COMPARTMENT_OCID
fi
