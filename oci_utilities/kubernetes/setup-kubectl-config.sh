#!/usr/bin/env bash

mkdir -p "$HOME/.kube"

oci ce cluster create-kubeconfig \
    --cluster-id ocid1.cluster.oc1.iad.aaaaaaaa4g2ljyjl2wkzcmt2rn2ym7mdlaz6jlrie6gfcrmsscbtbmfkymcq \
    --file "$HOME/.kube/config" \
    --region us-ashburn-1 \
    --token-version 2.0.0 \
    --kube-endpoint PRIVATE_ENDPOINT

export KUBECONFIG=$HOME/.kube/config

# shellcheck disable=SC2016
echo '
To use, you must export into your shell:
export KUBECONFIG=$HOME/.kube/config
'
