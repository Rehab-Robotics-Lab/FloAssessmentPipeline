#!/usr/bin/env bash

# set to use principal auth using dynamic group permissions
echo "setting permissions"
OCI_CLI_AUTH=instance_principal
export OCI_CLI_AUTH
