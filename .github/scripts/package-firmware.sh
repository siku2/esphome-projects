#!/usr/bin/env bash
# Copies the firmware of a compiled device into <out>/<name>/ next to the manifest that its update entity reads.
set -euo pipefail

device="$1"
version="$2"
out="$3"

name="bean-${device}"
build="devices/.esphome/build/${name}"
target="${out}/${name}"
chip_family="$(sed -n 's/^#define ESPHOME_VARIANT "\(.*\)"$/\1/p' "${build}/src/esphome/core/defines.h")"
commit="${GITHUB_SHA:-$(git rev-parse HEAD)}"

mkdir -p "${target}"
cp "${build}/build/firmware.ota.bin" "${build}/build/firmware.factory.bin" "${target}/"

jq --null-input \
  --arg name "${name}" \
  --arg version "${version}" \
  --arg chip_family "${chip_family}" \
  --arg md5 "$(md5sum "${target}/firmware.ota.bin" | cut -d ' ' -f 1)" \
  --arg summary "Built from ${commit:0:7}" \
  --arg release_url "https://github.com/siku2/esphome-projects/commit/${commit}" \
  '{
    name: $name,
    version: $version,
    home_assistant_domain: "esphome",
    builds: [{
      chipFamily: $chip_family,
      ota: {path: "firmware.ota.bin", md5: $md5, summary: $summary, release_url: $release_url},
      parts: [{path: "firmware.factory.bin", offset: 0}]
    }]
  }' > "${target}/manifest.json"
