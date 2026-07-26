#!/usr/bin/env bash

set -eu

SCRIPT_DIR="$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)"
. "$SCRIPT_DIR/common.sh"

CONFIG_FILE="/boot/firmware/config.txt"
CMDLINE_FILE="/boot/firmware/cmdline.txt"

install_if_changed() {
    source_file="$1"
    destination="$2"
    mode="$3"

    if sudo cmp -s "$source_file" "$destination"; then
        return
    fi

    backup_file "$destination"
    sudo install -o root -g root -m "$mode" "$source_file" "$destination"
}

require_root_tooling

[ -f "$CONFIG_FILE" ] || fail "$CONFIG_FILE not found."
[ -f "$CMDLINE_FILE" ] || fail "$CMDLINE_FILE not found."

tmp_config="$(mktemp)"
sudo cat "$CONFIG_FILE" > "$tmp_config"
if grep -q '^enable_uart=' "$tmp_config"; then
    sed -i 's/^enable_uart=.*/enable_uart=1/' "$tmp_config"
else
    printf '\n# Enable the SEBA STM32 UART link.\nenable_uart=1\n' >> "$tmp_config"
fi
install_if_changed "$tmp_config" "$CONFIG_FILE" 0644
rm -f "$tmp_config"

tmp_cmdline="$(mktemp)"
sudo cat "$CMDLINE_FILE" |
    tr ' ' '\n' |
    grep -Ev '^console=(serial0|ttyAMA0),[^[:space:]]+$' |
    awk 'NF' |
    paste -sd ' ' - > "$tmp_cmdline"

[ -s "$tmp_cmdline" ] ||
    fail "Refusing to write an empty kernel command line."

printf '\n' >> "$tmp_cmdline"
install_if_changed "$tmp_cmdline" "$CMDLINE_FILE" 0644
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
