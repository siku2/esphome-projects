FROM docker.io/debian:13@sha256:f324c7ff54321e8d9c588493a20244965938ce0aa50bbd1022d38010e9ffc4b1

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
