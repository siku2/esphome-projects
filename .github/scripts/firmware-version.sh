#!/usr/bin/env bash
# Prints a firmware version for a device that only changes when the code generated for it changes.
set -euo pipefail

device="$1"
cache="devices/.esphome"
data_dir="${cache}/version"
generated="${data_dir}/build/bean-${device}"

# A separate data dir keeps stale files of the cached build out of the hash and leaves that build untouched.
rm -rf "${data_dir}"
trap 'rm -rf "${data_dir}"' EXIT
mkdir -p "${data_dir}"
for dir in external_components font image packages pio_components tflite; do
  mkdir -p "${cache}/${dir}"
  ln -s "../${dir}" "${data_dir}/${dir}"
done

ESPHOME_DATA_DIR="${data_dir}" esphome -s firmware_version dev compile --only-generate "devices/${device}.yaml" >&2

{
  # These files hold the build time
  (cd "${generated}" && find . -type f ! -name build_info.json ! -name build_info_data.cpp -print0 \
    | LC_ALL=C sort --zero-terminated | xargs --null sha256sum)
  sha256sum flake.lock
} | sha256sum | cut -c1-12
