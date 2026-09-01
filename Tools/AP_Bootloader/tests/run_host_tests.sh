#!/usr/bin/env bash
set -euo pipefail

repo_root=$(git rev-parse --show-toplevel)
test_dir="${repo_root}/Tools/AP_Bootloader/tests"
build_dir=$(mktemp -d /tmp/ardupilot-abin-tests-XXXXXX)
trap 'rm -rf "${build_dir}"' EXIT

g++ -std=gnu++14 -Wall -Wextra -Werror \
    -I"${test_dir}/include" \
    -I"${repo_root}/Tools/AP_Bootloader" \
    "${test_dir}/test_abin_file.cpp" \
    "${repo_root}/Tools/AP_Bootloader/abin_file.cpp" \
    "${repo_root}/Tools/AP_Bootloader/abin_header.cpp" \
    "${repo_root}/Tools/AP_Bootloader/md5.cpp" \
    -o "${build_dir}/test_abin_file"

"${build_dir}/test_abin_file"
