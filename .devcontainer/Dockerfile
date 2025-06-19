FROM mcr.microsoft.com/devcontainers/base:bookworm@sha256:da67c59f82e057e0ccb81ec7d13fb50464c26f477638018e6de1b2623a008a3a

RUN <<EOF
# Install tools
set -eux

apt-get update
apt-get install -y pipx python3

for name in "esphome" "ruff" "uv"; do
    pipx install --system-site-packages "$name"
done
EOF
