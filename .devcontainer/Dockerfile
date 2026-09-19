FROM docker.io/debian:13@sha256:9cc080028c43b27d2074d63a5f9caf7166d731494965616c1a6d2827a004585c

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    <<EOF
set -euxo pipefail
export DEBIAN_FRONTEND=noninteractive
apt-get update
apt-get install -y --no-install-recommends --no-install-suggests \
    build-essential \
    cmake \
    git \
    pipx \
    python3 \
    sudo
EOF

ARG ESPHOME_VERSION=2026.5.1
ARG PLATFORMIO_VERSION=6.1.19
RUN \
    --mount=type=cache,target=/opt/pipx/.cache,sharing=locked \
    --mount=type=cache,target=/opt/pipx/py,sharing=locked \
    --mount=type=cache,target=/root/.cache,sharing=locked \
    --mount=type=tmpfs,target=/opt/pipx/logs \
    <<EOF
set -euxo pipefail
pipx install \
    --global \
    --system-site-packages \
    "esphome==${ESPHOME_VERSION}" \
    "platformio==${PLATFORMIO_VERSION}"
esphome --version
EOF
