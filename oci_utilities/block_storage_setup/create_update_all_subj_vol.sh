#!/usr/bin/env bash
set -o errexit
set -o pipefail

script="$(realpath "$0")"
scriptpath="$(dirname "$script")"


# shellcheck source=../includes/get_all_subj.sh
source "$scriptpath/../includes/get_all_subj.sh"

bash "$scriptpath/create_update_subj_vol.sh" "$subjects"
