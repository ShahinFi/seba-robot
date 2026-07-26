#!/usr/bin/env bash

set -eu

SCRIPT_DIR="$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)"
. "$SCRIPT_DIR/common.sh"

CONFIG_FILE="/boot/firmware/config.txt"
CMDLINE_FILE="/boot/firmware/cmdline.txt"

require_root_tooling

[ -f "$CONFIG_FILE" ] || fail "$CONFIG_FILE not found."
[ -f "$CMDLINE_FILE" ] || fail "$CMDLINE_FILE not found."

backup_file "$CONFIG_FILE"
backup_file "$CMDLINE_FILE"

if sudo grep -q '^enable_uart=' "$CONFIG_FILE"; then
    sudo sed -i 's/^enable_uart=.*/enable_uart=1/' "$CONFIG_FILE"
else
    printf '\n# Enable the SEBA STM32 UART link.\nenable_uart=1\n' |
        sudo tee -a "$CONFIG_FILE" >/dev/null
fi

tmp_cmdline="$(mktemp)"
sudo cat "$CMDLINE_FILE" |
    tr ' ' '\n' |
    grep -v '^console=serial0,115200$' |
    paste -sd ' ' - > "$tmp_cmdline"
sudo install -o root -g root -m 0644 "$tmp_cmdline" "$CMDLINE_FILE"
rm -f "$tmp_cmdline"

sudo usermod -aG dialout "$SEBA_INSTALL_USER"

info "UART configured for STM32 serial link."
info "Reboot or log out/in before relying on updated dialout permissions."
if [ -e "$SEBA_SERIAL" ]; then
    ls -l "$SEBA_SERIAL"
else
    info "$SEBA_SERIAL is not present yet. Reboot and check again."
    ls -l /dev/ttyAMA* /dev/ttyS* 2>/dev/null || true
fi

record_install_state
