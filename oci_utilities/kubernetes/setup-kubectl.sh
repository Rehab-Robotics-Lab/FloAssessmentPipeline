#!/usr/bin/env bash
set -o errexit
set -o pipefail

script="$(realpath "$0")"
scriptpath="$(dirname "$script")"
compartment_id=$(bash "$scriptpath/../functions/get_compartment_id.sh" 'flo')
kubernetes_version=$(oci ce cluster list \
    --compartment-id "$compartment_id" \
    --query 'data[0]."kubernetes-version"' \
    --raw-output )

echo "installing for kubernetes $kubernetes_version"

(cd "$HOME/Downloads" && curl -LO https://dl.k8s.io/release/"$kubernetes_version"/bin/linux/amd64/kubectl)
sudo install -o root -g root -m 0755 "$HOME/Downloads/kubectl" /usr/local/bin/kubectl
