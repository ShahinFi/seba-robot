#!/usr/bin/env bash

set -eu

SERVICE_NAME="seba-web.service"
SCRIPT_DIR="$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)"
REPO_DIR="$(dirname "$SCRIPT_DIR")"
TEMPLATE="$SCRIPT_DIR/systemd/$SERVICE_NAME.in"
SERVICE_FILE="/etc/systemd/system/$SERVICE_NAME"

SERVICE_USER="${SEBA_SERVICE_USER:-$(id -un)}"
SERIAL_PORT="${SEBA_SERIAL:-/dev/ttyAMA0}"
WEB_PORT="${SEBA_WEB_PORT:-8080}"

if [ -x "$REPO_DIR/.venv/bin/python3" ]; then
    PYTHON="$REPO_DIR/.venv/bin/python3"
else
    PYTHON="$(command -v python3)"
fi

tmp_file="$(mktemp)"
sed \
    -e "s|@USER@|$SERVICE_USER|g" \
    -e "s|@REPO_DIR@|$REPO_DIR|g" \
    -e "s|@SERIAL_PORT@|$SERIAL_PORT|g" \
    -e "s|@WEB_PORT@|$WEB_PORT|g" \
    -e "s|@PYTHON@|$PYTHON|g" \
    "$TEMPLATE" > "$tmp_file"

sudo install -m 0644 "$tmp_file" "$SERVICE_FILE"
rm -f "$tmp_file"

sudo systemctl daemon-reload
sudo systemctl enable "$SERVICE_NAME"
sudo systemctl restart "$SERVICE_NAME"
sudo systemctl --no-pager status "$SERVICE_NAME"
