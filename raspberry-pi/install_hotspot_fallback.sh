#!/usr/bin/env bash

set -eu

SERVICE_NAME="seba-hotspot-fallback.service"
SCRIPT_DIR="$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)"
REPO_DIR="$(dirname "$SCRIPT_DIR")"
TEMPLATE="$SCRIPT_DIR/systemd/$SERVICE_NAME.in"
SERVICE_FILE="/etc/systemd/system/$SERVICE_NAME"
HOTSPOT_CONNECTION="${SEBA_HOTSPOT_CONNECTION:-seba-hotspot}"

if ! command -v nmcli >/dev/null 2>&1; then
    echo "ERROR: nmcli is required. Install or enable NetworkManager." >&2
    exit 1
fi

if ! nmcli -t -f NAME connection show | grep -Fxq "$HOTSPOT_CONNECTION"; then
    echo "ERROR: hotspot connection '$HOTSPOT_CONNECTION' is not installed." >&2
    echo "Run: bash raspberry-pi/hotspot.sh install" >&2
    exit 1
fi

tmp_file="$(mktemp)"
sed \
    -e "s|@REPO_DIR@|$REPO_DIR|g" \
    "$TEMPLATE" > "$tmp_file"

sudo install -m 0644 "$tmp_file" "$SERVICE_FILE"
rm -f "$tmp_file"

sudo systemctl daemon-reload
sudo systemctl enable "$SERVICE_NAME"
sudo systemctl restart "$SERVICE_NAME"
sudo systemctl --no-pager status "$SERVICE_NAME"
