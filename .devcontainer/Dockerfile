FROM docker.io/debian:trixie-20260505@sha256:e2d08da6f42ef4b09b165d55528a12727aeed8240dc9edf888e3ec07e10ef9da

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
