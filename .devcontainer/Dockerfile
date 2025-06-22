FROM mcr.microsoft.com/devcontainers/base:ubuntu-24.04@sha256:601ef3f088ffd5ca63aee990a46712478d14dbf8927076d3118775a7b3fd813d

SHELL [ "/bin/bash", "-eux", "-c" ]

RUN <<EOF
# System dependencies

APT_DEPS=(
    "pipx"
    "software-properties-common"
)

apt-get update
apt-get install -y "${APT_DEPS[@]}"

add-apt-repository --yes ppa:kicad/kicad-9.0-releases
EOF

RUN <<EOF
# Install tools

APT_DEPS=(
    "kicad"
    "python3"
    "xvfb"
)
PIPX_DEPS=(
    "esphome"
    "interactivehtmlbom"
    "ruff"
    "uv"
)

apt-get update
apt-get install -y "${APT_DEPS[@]}"

for name in "${PIPX_DEPS[@]}"; do
    pipx install --system-site-packages "$name"
done
EOF
