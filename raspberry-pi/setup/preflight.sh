#!/usr/bin/env bash

set -eu

SCRIPT_DIR="$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)"
. "$SCRIPT_DIR/common.sh"

require_command sudo
require_command systemctl
require_command uname
require_command grep

grep -q '^ID=ubuntu$' /etc/os-release ||
    fail "Supported operating system is Ubuntu Server 24.04."

grep -q '^VERSION_ID="24.04"$' /etc/os-release ||
    fail "Supported Ubuntu version is 24.04."

[ "$(uname -m)" = "aarch64" ] ||
    fail "Supported architecture is aarch64."

model="$(tr -d '\0' </proc/device-tree/model 2>/dev/null || true)"
case "$model" in
    *"Raspberry Pi 5"*) ;;
    *) fail "Supported hardware is Raspberry Pi 5. Detected: ${model:-unknown}" ;;
esac

[ -d /run/systemd/system ] ||
    fail "systemd is required."

[ -f /boot/firmware/config.txt ] ||
    fail "/boot/firmware/config.txt not found."

[ -f /boot/firmware/cmdline.txt ] ||
    fail "/boot/firmware/cmdline.txt not found."

sudo -v

info "SEBA Raspberry Pi preflight: PASS"
