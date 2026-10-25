#!/usr/bin/env bash
# Prints a firmware version for a device that only changes when its build inputs change.
set -euo pipefail

device="$1"
{
  esphome config "devices/${device}.yaml"
  git rev-parse HEAD:components
  git ls-files --stage -- base boards projects ':!:*.yaml'
  git ls-files --stage -- flake.lock
} | sha256sum | cut -c1-12
