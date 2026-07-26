#!/usr/bin/env bash

set -eu

HOTSPOT_CONNECTION="${SEBA_HOTSPOT_CONNECTION:-seba-hotspot}"
HOTSPOT_SSID="${SEBA_HOTSPOT_SSID:-SEBA-ROBOT}"
HOTSPOT_IFACE="${SEBA_HOTSPOT_IFACE:-wlan0}"
HOTSPOT_IP="${SEBA_HOTSPOT_IP:-10.42.0.1/24}"
HOTSPOT_URL_IP="${HOTSPOT_IP%%/*}"
FALLBACK_WAIT_SECONDS="${SEBA_HOTSPOT_FALLBACK_WAIT:-45}"

usage() {
    cat <<EOF
Usage: bash raspberry-pi/hotspot.sh <install|up|down|status|fallback|remove>

Commands:
  install   Create or update the NetworkManager hotspot connection.
  up        Start the hotspot.
  down      Stop the hotspot.
  status    Show hotspot and network status.
  fallback  Start the hotspot only when normal network access is unavailable.
  remove    Delete the hotspot connection.

Environment:
  SEBA_HOTSPOT_PASSWORD    WPA password used by install.
  SEBA_HOTSPOT_SSID        Wi-Fi name. Default: SEBA-ROBOT.
  SEBA_HOTSPOT_IFACE       Wi-Fi interface. Default: wlan0.
  SEBA_HOTSPOT_IP          Hotspot address. Default: 10.42.0.1/24.
EOF
}

require_nmcli() {
    if ! command -v nmcli >/dev/null 2>&1; then
        echo "ERROR: nmcli is required. Install or enable NetworkManager." >&2
        exit 1
    fi
}

connection_exists() {
    nmcli -t -f NAME connection show |
        grep -Fxq "$HOTSPOT_CONNECTION"
}

hotspot_active() {
    nmcli -t -f NAME connection show --active |
        grep -Fxq "$HOTSPOT_CONNECTION"
}

read_password() {
    if [ "${SEBA_HOTSPOT_PASSWORD:-}" ]; then
        printf '%s' "$SEBA_HOTSPOT_PASSWORD"
        return
    fi

    if [ ! -t 0 ]; then
        echo "ERROR: set SEBA_HOTSPOT_PASSWORD for non-interactive install." >&2
        exit 1
    fi

    printf "Hotspot password for %s: " "$HOTSPOT_SSID" >&2
    trap 'stty echo; printf "\n" >&2; exit 1' INT TERM
    stty -echo
    read -r password
    stty echo
    trap - INT TERM
    printf '\n' >&2
    printf '%s' "$password"
}

install_hotspot() {
    require_nmcli

    password="$(read_password)"

    if [ "${#password}" -lt 8 ] || [ "${#password}" -gt 63 ]; then
        echo "ERROR: hotspot password must be 8 to 63 characters." >&2
        exit 1
    fi

    if connection_exists; then
        sudo nmcli connection delete "$HOTSPOT_CONNECTION" >/dev/null
    fi

    sudo nmcli connection add \
        type wifi \
        ifname "$HOTSPOT_IFACE" \
        con-name "$HOTSPOT_CONNECTION" \
        autoconnect no \
        ssid "$HOTSPOT_SSID" >/dev/null

    sudo nmcli connection modify "$HOTSPOT_CONNECTION" \
        802-11-wireless.mode ap \
        802-11-wireless.band bg \
        ipv4.method shared \
        ipv4.addresses "$HOTSPOT_IP" \
        ipv6.method disabled \
        wifi-sec.key-mgmt wpa-psk \
        wifi-sec.psk "$password"

    echo "Installed hotspot connection: $HOTSPOT_CONNECTION"
    echo "SSID: $HOTSPOT_SSID"
    echo "Address: $HOTSPOT_URL_IP"
}

start_hotspot() {
    require_nmcli

    if ! connection_exists; then
        echo "ERROR: hotspot is not installed. Run: bash raspberry-pi/hotspot.sh install" >&2
        exit 1
    fi

    sudo nmcli connection up "$HOTSPOT_CONNECTION" >/dev/null

    echo "Hotspot started."
    echo "Connect to Wi-Fi: $HOTSPOT_SSID"
    echo "Control: http://$HOTSPOT_URL_IP:8080/control"
    echo "Tuner:   http://$HOTSPOT_URL_IP:8080/tuner"
}

stop_hotspot() {
    require_nmcli

    if hotspot_active; then
        sudo nmcli connection down "$HOTSPOT_CONNECTION" >/dev/null
        echo "Hotspot stopped."
    else
        echo "Hotspot is not active."
    fi
}

remove_hotspot() {
    require_nmcli

    if connection_exists; then
        sudo nmcli connection delete "$HOTSPOT_CONNECTION" >/dev/null
        echo "Removed hotspot connection: $HOTSPOT_CONNECTION"
    else
        echo "Hotspot connection is not installed."
    fi
}

active_connection_for_device() {
    target_device="$1"

    while IFS=: read -r name device; do
        if [ "$device" = "$target_device" ]; then
            printf '%s\n' "$name"
            return 0
        fi
    done < <(nmcli -t -f NAME,DEVICE connection show --active)

    return 1
}

normal_network_available() {
    while read -r route; do
        gateway="$(printf '%s\n' "$route" | sed -n 's/.* via \([^ ]*\).*/\1/p')"
        device="$(printf '%s\n' "$route" | sed -n 's/.* dev \([^ ]*\).*/\1/p')"

        if [ -z "$gateway" ] || [ -z "$device" ]; then
            continue
        fi

        connection="$(active_connection_for_device "$device" || true)"

        if [ "$connection" = "$HOTSPOT_CONNECTION" ]; then
            continue
        fi

        if ping -c 1 -W 1 "$gateway" >/dev/null 2>&1; then
            return 0
        fi
    done < <(ip -4 route show default)

    return 1
}

fallback_hotspot() {
    require_nmcli

    sleep "$FALLBACK_WAIT_SECONDS"

    if normal_network_available; then
        echo "Normal network is available. Hotspot fallback not started."
        exit 0
    fi

    echo "Normal network is unavailable. Starting hotspot fallback."
    start_hotspot
}

show_status() {
    require_nmcli

    if connection_exists; then
        echo "Installed: yes"
    else
        echo "Installed: no"
    fi

    if hotspot_active; then
        echo "Active: yes"
        echo "SSID: $HOTSPOT_SSID"
        echo "Control: http://$HOTSPOT_URL_IP:8080/control"
        echo "Tuner:   http://$HOTSPOT_URL_IP:8080/tuner"
    else
        echo "Active: no"
    fi

    echo ""
    echo "Active NetworkManager connections:"
    nmcli connection show --active
}

case "${1:-}" in
    install)
        install_hotspot
        ;;
    up)
        start_hotspot
        ;;
    down)
        stop_hotspot
        ;;
    status)
        show_status
        ;;
    fallback)
        fallback_hotspot
        ;;
    remove)
        remove_hotspot
        ;;
    -h|--help|help)
        usage
        ;;
    *)
        usage >&2
        exit 1
        ;;
esac
