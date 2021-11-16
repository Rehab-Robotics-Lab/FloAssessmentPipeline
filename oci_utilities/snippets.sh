#!/bin/bash

## Not meant to be run. just some snippets that are useful for running code
sudo dnf install -y tmux

OCI_CLI_AUTH=instance_principal
export OCI_CLI_AUTH

sudo dnf -y install oraclelinux-developer-release-el8
sudo dnf -y install python36-oci-cli

PROJECT_COMPARTMENT_OCID='ocid1.compartment.oc1..aaaaaaaadznuoh3ntsva2jsj453wwmemd4t2k5rnwniuzkliq7evffxgprua'
export PROJECT_COMPARTMENT_OCID

mkdir -p manual_jobs
oci os object get -bn 'rrl-flo-run' --file 'manual_jobs/run.sh' --name 'manual_jobs/run.sh'

oci-token os object put -bn 'rrl-flo-run' --file 'run.sh' --name 'manual_jobs/run.sh'

oci-token os object put -bn 'rrl-flo-run' --file 'run.sh' --name 'manual_jobs/run.sh'

oci os object bulk-download -bn 'rrl-flo-run' --download-dir '.'
