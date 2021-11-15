echo "getting block volume id"

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

volume_id=$(
    oci bv volume list \
        --compartment-id "$compartment" \
        --display-name "flo_data/$subject_padded" \
        --query 'data[? "lifecycle-state" == `"AVAILABLE"`] | [0].id' \
        --raw-output
)
echo "block storage volume ocid: $volume_id"
