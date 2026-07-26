#!/usr/bin/env bash

set -eu

SCRIPT_DIR="$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)"
. "$SCRIPT_DIR/common.sh"

require_root_tooling

sudo apt update
sudo apt install -y \
    git \
    python3 \
    python3-venv \
    python3-pip \
    network-manager \
    iw \
    rfkill \
    curl

sudo systemctl enable --now NetworkManager

cd "$REPO_DIR"

if [ ! -d ".venv" ]; then
    python3 -m venv .venv
fi

. .venv/bin/activate
python3 -m pip install --upgrade pip
python3 -m pip install -r raspberry-pi/requirements.txt

record_install_state
