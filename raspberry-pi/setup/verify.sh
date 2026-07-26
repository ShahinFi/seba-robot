#!/usr/bin/env bash

set -eu

SCRIPT_DIR="$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)"
. "$SCRIPT_DIR/common.sh"

failures=""

check() {
    name="$1"
    shift
    if "$@" >/dev/null 2>&1; then
        return 0
    fi

    failures="${failures}${name}
"
    return 1
}

check_output() {
    name="$1"
    expected="$2"
    shift 2
    output="$("$@" 2>/dev/null || true)"
    if printf '%s' "$output" | grep -Eq "$expected"; then
        return 0
    fi

    failures="${failures}${name}
"
    return 1
}

check "Ubuntu 24.04" grep -q '24\.04' /etc/os-release
check "aarch64 architecture" sh -c '[ "$(uname -m)" = "aarch64" ]'
check "NetworkManager active" systemctl is-active --quiet NetworkManager
check "wlan0 present" sh -c "nmcli -t -f DEVICE device status | grep -Fxq '$SEBA_HOTSPOT_IFACE'"
check "systemd-networkd not managing wlan0" sh -c "! networkctl status '$SEBA_HOTSPOT_IFACE' --no-pager 2>/dev/null | grep -E 'Network File: +/.+\\.network'"
if [ -n "$SEBA_WIFI_CONNECTION" ]; then
    check "normal Wi-Fi profile exists" nmcli connection show "$SEBA_WIFI_CONNECTION"
    check_output "normal Wi-Fi autoconnect enabled" '^yes$' nmcli -g connection.autoconnect connection show "$SEBA_WIFI_CONNECTION"
fi
check "hotspot profile exists" nmcli connection show "$SEBA_HOTSPOT_CONNECTION"
check_output "hotspot AP mode" '^ap$' nmcli -g 802-11-wireless.mode connection show "$SEBA_HOTSPOT_CONNECTION"
check_output "hotspot autoconnect disabled" '^no$' nmcli -g connection.autoconnect connection show "$SEBA_HOTSPOT_CONNECTION"
check_output "hotspot channel configured" '^[0-9]+$' nmcli -g 802-11-wireless.channel connection show "$SEBA_HOTSPOT_CONNECTION"
check_output "hotspot powersave disabled" '^2' nmcli -g 802-11-wireless.powersave connection show "$SEBA_HOTSPOT_CONNECTION"
check_output "hotspot PMF optional" '^2' nmcli -g 802-11-wireless-security.pmf connection show "$SEBA_HOTSPOT_CONNECTION"
check_output "hotspot IPv4 shared" '^shared$' nmcli -g ipv4.method connection show "$SEBA_HOTSPOT_CONNECTION"
check "Wi-Fi country service enabled" systemctl is-enabled --quiet seba-wifi-country.service
check_output "global regulatory country configured" "country ${SEBA_WIFI_COUNTRY}:" iw reg get
check "Ethernet recovery address present" sh -c "ip -4 addr show '$SEBA_ETH_IFACE' | grep -q '${SEBA_ETH_ADDRESS%%/*}'"
check "UART device present" test -e "$SEBA_SERIAL"
check "install user in dialout" sh -c "id -nG '$SEBA_INSTALL_USER' | tr ' ' '\n' | grep -Fxq dialout"
check "Python virtual environment valid" test -x "$REPO_DIR/.venv/bin/python3"
check "web service enabled" systemctl is-enabled --quiet seba-web.service
check "web service active" systemctl is-active --quiet seba-web.service
check "fallback service enabled" systemctl is-enabled --quiet seba-hotspot-fallback.service
check "control endpoint reachable locally" curl -fsS "http://127.0.0.1:${SEBA_WEB_PORT}/control"

if [ -n "$failures" ]; then
    printf 'SEBA Raspberry Pi setup: FAIL\n'
    printf '%s' "$failures" | sed '/^$/d; s/^/- /'
    exit 1
fi

printf 'SEBA Raspberry Pi setup: PASS\n'
