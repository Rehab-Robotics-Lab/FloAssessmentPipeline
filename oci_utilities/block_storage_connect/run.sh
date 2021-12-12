#!/usr/bin/env bash
set -o errexit
set -o pipefail

script="$(realpath "$0")"
scriptpath="$(dirname "$script")"

# shellcheck source=../includes/check_version.sh
source "$scriptpath/../includes/check_version.sh"

# shellcheck source=../includes/set_permissions_job.sh
source "$scriptpath/../includes/set_permissions_job.sh"

# shellcheck source=../includes/parse_input_subj_no.sh
source "$scriptpath/../includes/parse_input_subj_no.sh"

# shellcheck source=../includes/get_instance_info.sh
source "$scriptpath/../includes/get_instance_info.sh"

# shellcheck source=../includes/get_block_volume_id.sh
source "$scriptpath/../includes/get_block_volume_id.sh"

# shellcheck source=../includes/attach_block_volume.sh
source "$scriptpath/../includes/attach_block_volume.sh"

# shellcheck source=../includes/mount_block_volume.sh
source "$scriptpath/../includes/mount_block_volume.sh"

sleep 5

read -r -p 'Press any key to disconnect block volume. Press ctrl-z to send to back. Type fg to bring back to front'

echo "job complete"
