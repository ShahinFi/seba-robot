#!/usr/bin/env bash

set -eu

HOTSPOT_CONNECTION="${SEBA_HOTSPOT_CONNECTION:-seba-hotspot}"
HOTSPOT_SSID="${SEBA_HOTSPOT_SSID:-SEBA-ROBOT}"
HOTSPOT_IFACE="${SEBA_HOTSPOT_IFACE:-wlan0}"
HOTSPOT_IP="${SEBA_HOTSPOT_IP:-10.42.0.1/24}"
HOTSPOT_URL_IP="${HOTSPOT_IP%%/*}"
HOTSPOT_CHANNEL="${SEBA_HOTSPOT_CHANNEL:-11}"
SCRIPT_DIR="$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)"
LOG_FILE="${SEBA_HOTSPOT_LOG:-$SCRIPT_DIR/hotspot.log}"
FALLBACK_WAIT_SECONDS="${SEBA_HOTSPOT_FALLBACK_WAIT:-45}"
MONITOR_INTERVAL_SECONDS="${SEBA_HOTSPOT_MONITOR_INTERVAL:-5}"
NORMAL_NETWORK_REASON="not checked"

usage() {
    cat <<EOF
Usage: bash raspberry-pi/hotspot.sh <install|up|down|status|fallback|monitor|remove>

Commands:
  install   Create or update the NetworkManager hotspot connection.
  up        Start the hotspot.
  down      Stop the hotspot.
  status    Show hotspot and network status.
  fallback  Start the hotspot only when normal network access is unavailable.
  monitor   Watch normal network access and start the hotspot when it is lost.
  remove    Delete the hotspot connection.

Environment:
  SEBA_HOTSPOT_PASSWORD    WPA password used by install.
  SEBA_HOTSPOT_SSID        Wi-Fi name. Default: SEBA-ROBOT.
  SEBA_HOTSPOT_IFACE       Wi-Fi interface. Default: wlan0.
  SEBA_HOTSPOT_IP          Hotspot address. Default: 10.42.0.1/24.
  SEBA_HOTSPOT_CHANNEL     2.4 GHz channel. Default: 11.
  SEBA_HOTSPOT_FALLBACK_WAIT
                           Seconds to wait before fallback/monitor checks.
  SEBA_HOTSPOT_MONITOR_INTERVAL
                           Seconds between monitor checks. Default: 5.
  SEBA_HOTSPOT_LOG         Diagnostic log path.
EOF
}

log_message() {
    line="$(date '+%Y-%m-%d %H:%M:%S') $*"
    printf '%s\n' "$line"
    printf '%s\n' "$line" >> "$LOG_FILE" 2>/dev/null || true
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

    if ! connection_exists; then
        sudo nmcli connection add \
            type wifi \
            ifname "$HOTSPOT_IFACE" \
            con-name "$HOTSPOT_CONNECTION" \
            autoconnect no \
            ssid "$HOTSPOT_SSID" >/dev/null
    fi

    sudo nmcli connection modify "$HOTSPOT_CONNECTION" \
        connection.autoconnect no \
        connection.interface-name "$HOTSPOT_IFACE" \
        802-11-wireless.ssid "$HOTSPOT_SSID" \
        802-11-wireless.mode ap \
        802-11-wireless.band bg \
        802-11-wireless.channel "$HOTSPOT_CHANNEL" \
        802-11-wireless.hidden no \
        802-11-wireless.powersave 2 \
        ipv4.method shared \
        ipv4.addresses "$HOTSPOT_IP" \
        ipv6.method disabled \
        wifi-sec.key-mgmt wpa-psk \
        wifi-sec.psk "$password" \
        wifi-sec.proto rsn \
        wifi-sec.pairwise ccmp \
        wifi-sec.group ccmp \
        wifi-sec.pmf 2

    echo "Installed hotspot connection: $HOTSPOT_CONNECTION"
    echo "SSID: $HOTSPOT_SSID"
    echo "Channel: $HOTSPOT_CHANNEL"
    echo "Address: $HOTSPOT_URL_IP"
}

start_hotspot() {
    require_nmcli

    if ! connection_exists; then
        echo "ERROR: hotspot is not installed. Run: bash raspberry-pi/hotspot.sh install" >&2
        exit 1
    fi

    if ! output="$(sudo nmcli connection up "$HOTSPOT_CONNECTION" 2>&1)"; then
        log_message "ERROR: failed to start hotspot: $output"
        echo "ERROR: failed to start hotspot: $output" >&2
        exit 1
    fi

    log_message "Hotspot started. $output"
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
    NORMAL_NETWORK_REASON="no default route"

    while read -r route; do
        gateway="$(printf '%s\n' "$route" | sed -n 's/.* via \([^ ]*\).*/\1/p')"
        device="$(printf '%s\n' "$route" | sed -n 's/.* dev \([^ ]*\).*/\1/p')"

        if [ -z "$gateway" ] || [ -z "$device" ]; then
            NORMAL_NETWORK_REASON="default route without gateway/device: $route"
            continue
        fi

        connection="$(active_connection_for_device "$device" || true)"

        if [ -z "$connection" ]; then
            if [ "$device" = "$HOTSPOT_IFACE" ]; then
                NORMAL_NETWORK_REASON="default route has no active NetworkManager connection: $route"
                continue
            fi

            connection="unmanaged"
        fi

        if [ "$connection" = "$HOTSPOT_CONNECTION" ]; then
            NORMAL_NETWORK_REASON="default route belongs to hotspot connection $HOTSPOT_CONNECTION"
            continue
        fi

        if ping -c 1 -W 1 "$gateway" >/dev/null 2>&1; then
            NORMAL_NETWORK_REASON="gateway reachable route='$route' connection='$connection'"
            return 0
        fi

        NORMAL_NETWORK_REASON="gateway unreachable route='$route' connection='$connection'"
    done < <(ip -4 route show default)

    return 1
}

fallback_hotspot() {
    require_nmcli

    sleep "$FALLBACK_WAIT_SECONDS"

    if normal_network_available; then
        log_message "Normal network is available. Hotspot fallback not started. $NORMAL_NETWORK_REASON"
        echo "Normal network is available. Hotspot fallback not started."
        exit 0
    fi

    log_message "Normal network is unavailable. Starting hotspot fallback. $NORMAL_NETWORK_REASON"
    echo "Normal network is unavailable. Starting hotspot fallback."
    start_hotspot
}

monitor_hotspot() {
    require_nmcli

    if ! connection_exists; then
        log_message "ERROR: hotspot is not installed. Run: bash raspberry-pi/hotspot.sh install"
        echo "ERROR: hotspot is not installed. Run: bash raspberry-pi/hotspot.sh install" >&2
        exit 1
    fi

    log_message "Hotspot monitor started. wait=${FALLBACK_WAIT_SECONDS}s interval=${MONITOR_INTERVAL_SECONDS}s connection=$HOTSPOT_CONNECTION iface=$HOTSPOT_IFACE"
    sleep "$FALLBACK_WAIT_SECONDS"

    while true; do
        if hotspot_active; then
            log_message "Hotspot is active. Network monitor exiting."
            echo "Hotspot is active. Network monitor exiting."
            exit 0
        fi

        if normal_network_available; then
            log_message "Normal network is available. Hotspot monitor waiting. $NORMAL_NETWORK_REASON"
            echo "Normal network is available. Hotspot monitor waiting."
            sleep "$MONITOR_INTERVAL_SECONDS"
            continue
        fi

        log_message "Normal network is unavailable. Starting hotspot and staying in hotspot mode. $NORMAL_NETWORK_REASON"
        echo "Normal network is unavailable. Starting hotspot and staying in hotspot mode."
        start_hotspot
        exit 0
    done
}

show_status() {
    require_nmcli

    echo "Log: $LOG_FILE"

    if command -v systemctl >/dev/null 2>&1; then
        echo "NetworkManager: $(systemctl is-active NetworkManager 2>/dev/null || true)"
    fi

    echo "Wi-Fi device:"
    nmcli -t -f DEVICE,TYPE,STATE,CONNECTION device status |
        grep "^$HOTSPOT_IFACE:" || echo "$HOTSPOT_IFACE:not-found"

    echo ""
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
    if normal_network_available; then
        echo "Normal network: available ($NORMAL_NETWORK_REASON)"
    else
        echo "Normal network: unavailable ($NORMAL_NETWORK_REASON)"
    fi

    echo ""
    echo "Default routes:"
    ip -4 route show default || true

    echo ""
    echo "Active NetworkManager connections:"
    nmcli connection show --active

    echo ""
    echo "Recent hotspot log:"
    if [ -f "$LOG_FILE" ]; then
        tail -30 "$LOG_FILE"
    else
        echo "no log file"
    fi
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
    monitor)
        monitor_hotspot
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
