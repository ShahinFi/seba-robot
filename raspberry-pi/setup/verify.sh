#!/usr/bin/env bash

set -eu

SCRIPT_DIR="$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)"
. "$SCRIPT_DIR/common.sh"

CLOUD_INIT_DISABLE_FILE="/etc/cloud/cloud.cfg.d/99-disable-network-config.cfg"
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

check_exact_output() {
    name="$1"
    expected="$2"
    shift 2
    output="$("$@" 2>/dev/null || true)"
    if [ "$output" = "$expected" ]; then
        return 0
    fi

    failures="${failures}${name}
"
    return 1
}

check "Ubuntu 24.04" sh -c \
    "grep -q '^ID=ubuntu$' /etc/os-release && grep -q '^VERSION_ID=\"24.04\"$' /etc/os-release"
check "aarch64 architecture" sh -c '[ "$(uname -m)" = "aarch64" ]'
check "Raspberry Pi 5" sh -c \
    "tr -d '\0' </proc/device-tree/model | grep -q 'Raspberry Pi 5'"
check "NetworkManager active" systemctl is-active --quiet NetworkManager
check "wlan0 present" sh -c "nmcli -t -f DEVICE device status | grep -Fxq '$SEBA_HOTSPOT_IFACE'"
check "systemd-networkd not managing wlan0" sh -c \
    "! networkctl status '$SEBA_HOTSPOT_IFACE' --no-pager 2>/dev/null | grep -E 'Network File: +/.+\\.network'"

check "SEBA Netplan file exists" test -f "$SEBA_NETPLAN_FILE"
check_output "SEBA Netplan owner" '^root:root$' stat -c '%U:%G' "$SEBA_NETPLAN_FILE"
check_output "SEBA Netplan mode" '^600$' stat -c '%a' "$SEBA_NETPLAN_FILE"
check "Netplan generates" sudo netplan generate
check "cloud-init network disabled" test -f "$CLOUD_INIT_DISABLE_FILE"
check "UART enabled in firmware config" sh -c \
    "grep -Eq '^enable_uart=1$' /boot/firmware/config.txt"
check "robot UART not used as kernel console" sh -c \
    "! tr ' ' '\n' </boot/firmware/cmdline.txt | grep -Eq '^console=(serial0|ttyAMA0),[^[:space:]]+$'"
check "no other Netplan file owns Ethernet recovery" sh -c "
    for file in /etc/netplan/*.yaml; do
        [ -e \"\$file\" ] || continue
        [ \"\$file\" = '$SEBA_NETPLAN_FILE' ] && continue
        ! sudo grep -Eq '^[[:space:]]+$SEBA_ETH_IFACE:' \"\$file\"
    done
"

if [ -n "$SEBA_WIFI_CONNECTION" ]; then
    check "normal Wi-Fi profile exists" nmcli connection show "$SEBA_WIFI_CONNECTION"
    check_exact_output "normal Wi-Fi autoconnect enabled" 'yes' \
        nmcli -g connection.autoconnect connection show "$SEBA_WIFI_CONNECTION"
    check_output "normal Wi-Fi powersave disabled" '^2' \
        nmcli -g 802-11-wireless.powersave connection show "$SEBA_WIFI_CONNECTION"
fi

check "hotspot profile exists" nmcli connection show "$SEBA_HOTSPOT_CONNECTION"
check_exact_output "hotspot interface" "$SEBA_HOTSPOT_IFACE" \
    nmcli -g connection.interface-name connection show "$SEBA_HOTSPOT_CONNECTION"
check_exact_output "hotspot SSID" "$SEBA_HOTSPOT_SSID" \
    nmcli -g 802-11-wireless.ssid connection show "$SEBA_HOTSPOT_CONNECTION"
check_exact_output "hotspot AP mode" 'ap' \
    nmcli -g 802-11-wireless.mode connection show "$SEBA_HOTSPOT_CONNECTION"
check_exact_output "hotspot band" 'bg' \
    nmcli -g 802-11-wireless.band connection show "$SEBA_HOTSPOT_CONNECTION"
check_exact_output "hotspot autoconnect disabled" 'no' \
    nmcli -g connection.autoconnect connection show "$SEBA_HOTSPOT_CONNECTION"
check_exact_output "hotspot channel" "$SEBA_HOTSPOT_CHANNEL" \
    nmcli -g 802-11-wireless.channel connection show "$SEBA_HOTSPOT_CONNECTION"
check_output "hotspot powersave disabled" '^2' \
    nmcli -g 802-11-wireless.powersave connection show "$SEBA_HOTSPOT_CONNECTION"
check_exact_output "hotspot WPA2 key management" 'wpa-psk' \
    nmcli -g 802-11-wireless-security.key-mgmt connection show "$SEBA_HOTSPOT_CONNECTION"
check_output "hotspot RSN protocol" '(^|,)rsn(,|$)' \
    nmcli -g 802-11-wireless-security.proto connection show "$SEBA_HOTSPOT_CONNECTION"
check_output "hotspot pairwise CCMP" 'ccmp' \
    nmcli -g 802-11-wireless-security.pairwise connection show "$SEBA_HOTSPOT_CONNECTION"
check_output "hotspot group CCMP" 'ccmp' \
    nmcli -g 802-11-wireless-security.group connection show "$SEBA_HOTSPOT_CONNECTION"
check_output "hotspot PMF optional" '^2' \
    nmcli -g 802-11-wireless-security.pmf connection show "$SEBA_HOTSPOT_CONNECTION"
check_exact_output "hotspot IPv4 shared" 'shared' \
    nmcli -g ipv4.method connection show "$SEBA_HOTSPOT_CONNECTION"
check_exact_output "hotspot IPv4 address" "$SEBA_HOTSPOT_IP" \
    nmcli -g ipv4.addresses connection show "$SEBA_HOTSPOT_CONNECTION"
check_exact_output "hotspot IPv6 disabled" 'disabled' \
    nmcli -g ipv6.method connection show "$SEBA_HOTSPOT_CONNECTION"

check "Wi-Fi country service enabled" systemctl is-enabled --quiet seba-wifi-country.service
check "Wi-Fi country service successful" systemctl is-active --quiet seba-wifi-country.service
check_output "global regulatory country configured" "country ${SEBA_WIFI_COUNTRY}:" iw reg get
check "Ethernet recovery address present" sh -c \
    "ip -4 addr show '$SEBA_ETH_IFACE' | grep -q '${SEBA_ETH_ADDRESS%%/*}'"
check "UART device present" test -e "$SEBA_SERIAL"
check "install user in dialout" sh -c \
    "id -nG '$SEBA_INSTALL_USER' | tr ' ' '\n' | grep -Fxq dialout"
check "Python virtual environment valid" test -x "$REPO_DIR/.venv/bin/python3"
check "pyserial import" "$REPO_DIR/.venv/bin/python3" -c 'import serial'
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
