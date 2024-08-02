#!/usr/bin/env bash

sudo bash ./.devcontainer/on-create-root.sh

pipx ensurepath
pipx install esphome
