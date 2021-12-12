#!/usr/bin/env bash
set -o errexit
set -o pipefail

oci iam compartment list --query "data[? contains(\"name\", '$1')]| [0].id" --raw-output
