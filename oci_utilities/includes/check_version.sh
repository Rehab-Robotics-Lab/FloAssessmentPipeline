#!/usr/bin/env bash

echo "checking version"
echo "OS version: $(cat /etc/oracle-release)"

echo "CLI Version: $(oci -v)"
