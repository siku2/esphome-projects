FROM mcr.microsoft.com/devcontainers/base:bookworm

RUN <<EOF
# Install tools
set -eux

apt-get update
apt-get install -y pipx python3

for name in "uv" "esphome"; do
    pipx install --system-site-packages "$name"
done
EOF
