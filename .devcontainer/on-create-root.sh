#!/usr/bin/env bash

export DEBIAN_FRONTEND=noninteractive

apt-get update
apt-get install -y pipx python3

usermod -a -G dialout vscode
chown :dialout /dev/tty*
chmod g+rw /dev/tty*
