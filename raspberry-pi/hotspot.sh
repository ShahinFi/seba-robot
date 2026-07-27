#!/usr/bin/env bash

set -eu

SCRIPT_DIR="$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

save_explicit_value() {
    name="$1"
    if [ "${!name+x}" ]; then
        printf -v "_EXPLICIT_$name" '%s' "${!name}"
        printf -v "_HAS_EXPLICIT_$name" '%s' 1
    fi
}

restore_explicit_value() {
    name="$1"
    has_name="_HAS_EXPLICIT_$name"
    value_name="_EXPLICIT_$name"

    if [ "${!has_name:-0}" = "1" ]; then
        printf -v "$name" '%s' "${!value_name}"
        export "$name"
    fi
}

for name in \
    SEBA_WIFI_CONNECTION \
    SEBA_HOTSPOT_CONNECTION \
    SEBA_HOTSPOT_SSID \
    SEBA_HOTSPOT_IFACE \
    SEBA_HOTSPOT_IP \
    SEBA_HOTSPOT_CHANNEL \
    SEBA_WEB_PORT \
    SEBA_HOTSPOT_FALLBACK_WAIT \
    SEBA_HOTSPOT_MONITOR_INTERVAL
do
    save_explicit_value "$name"
done

INSTALL_STATE_FILE="${SEBA_INSTALL_STATE_FILE:-$SCRIPT_DIR/.install-state}"
if [ -f "$INSTALL_STATE_FILE" ]; then
    . "$INSTALL_STATE_FILE"
fi

# Explicit environment values override recorded setup state so one command can
# temporarily change a hotspot or Wi-Fi setting without editing install files.
for name in \
    SEBA_WIFI_CONNECTION \
    SEBA_HOTSPOT_CONNECTION \
    SEBA_HOTSPOT_SSID \
    SEBA_HOTSPOT_IFACE \
    SEBA_HOTSPOT_IP \
    SEBA_HOTSPOT_CHANNEL \
    SEBA_WEB_PORT \
    SEBA_HOTSPOT_FALLBACK_WAIT \
    SEBA_HOTSPOT_MONITOR_INTERVAL
do
    restore_explicit_value "$name"
done

HOTSPOT_CONNECTION="${SEBA_HOTSPOT_CONNECTION:-seba-hotspot}"
NORMAL_CONNECTION="${SEBA_WIFI_CONNECTION:-seba-router}"
HOTSPOT_SSID="${SEBA_HOTSPOT_SSID:-SEBA-ROBOT}"
HOTSPOT_IFACE="${SEBA_HOTSPOT_IFACE:-wlan0}"
HOTSPOT_IP="${SEBA_HOTSPOT_IP:-10.42.0.1/24}"
HOTSPOT_URL_IP="${HOTSPOT_IP%%/*}"
HOTSPOT_CHANNEL="${SEBA_HOTSPOT_CHANNEL:-11}"
WEB_PORT="${SEBA_WEB_PORT:-8080}"
FALLBACK_WAIT_SECONDS="${SEBA_HOTSPOT_FALLBACK_WAIT:-45}"
MONITOR_INTERVAL_SECONDS="${SEBA_HOTSPOT_MONITOR_INTERVAL:-10}"
NORMAL_NETWORK_REASON="not checked"
TRANSITION_MARKER="/run/seba-hotspot-transition"
TRANSITION_LOCK="/run/lock/seba-hotspot-transition.lock"

usage() {
    cat <<EOF
Usage: bash raspberry-pi/hotspot.sh <install|up|down|status|fallback|monitor|remove>

Commands:
  install   Create or update the NetworkManager hotspot connection.
  up        Start the hotspot.
  down      Leave hotspot mode and reconnect to normal Wi-Fi.
  status    Show hotspot and network status.
  fallback  Start the hotspot only when normal network access is unavailable.
  monitor   Watch normal network access and start the hotspot when it is lost.
  remove    Delete the hotspot connection.

Environment:
  SEBA_WIFI_CONNECTION     Normal Wi-Fi profile. Default: seba-router.
  SEBA_HOTSPOT_PASSWORD    WPA password used by install.
  SEBA_HOTSPOT_SSID        Wi-Fi name. Default: SEBA-ROBOT.
  SEBA_HOTSPOT_IFACE       Wi-Fi interface. Default: wlan0.
  SEBA_HOTSPOT_IP          Hotspot address. Default: 10.42.0.1/24.
  SEBA_HOTSPOT_CHANNEL     2.4 GHz channel. Default: 11.
  SEBA_WEB_PORT            Web server port printed in URLs. Default: 8080.
  SEBA_HOTSPOT_FALLBACK_WAIT
                           Seconds to wait before fallback/monitor checks.
  SEBA_HOTSPOT_MONITOR_INTERVAL
                           Seconds between monitor checks. Default: 10.
EOF
}

log_message() {
    printf '%s\n' "$*"
}

error_message() {
    printf 'ERROR: %s\n' "$*" >&2
}

require_nmcli() {
    if ! command -v nmcli >/dev/null 2>&1; then
        error_message "nmcli is required. Install or enable NetworkManager."
        exit 1
    fi
}

require_command() {
    if ! command -v "$1" >/dev/null 2>&1; then
        error_message "$1 is required."
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
        error_message "set SEBA_HOTSPOT_PASSWORD for non-interactive install."
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
        error_message "hotspot password must be 8 to 63 characters."
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
        connection.interface-name "$HOTSPOT_IFACE" \
        connection.autoconnect no \
        802-11-wireless.ssid "$HOTSPOT_SSID" \
        802-11-wireless.mode ap \
        802-11-wireless.band bg \
        802-11-wireless.channel "$HOTSPOT_CHANNEL" \
        802-11-wireless.powersave 2 \
        802-11-wireless-security.key-mgmt wpa-psk \
        802-11-wireless-security.psk "$password" \
        802-11-wireless-security.proto rsn \
        802-11-wireless-security.pairwise ccmp \
        802-11-wireless-security.group ccmp \
        802-11-wireless-security.pmf 2 \
        ipv4.method shared \
        ipv4.addresses "$HOTSPOT_IP" \
        ipv6.method disabled

    echo "Installed hotspot connection: $HOTSPOT_CONNECTION"
    echo "SSID: $HOTSPOT_SSID"
    echo "Channel: $HOTSPOT_CHANNEL"
    echo "Address: $HOTSPOT_URL_IP"
}

start_hotspot() {
    require_nmcli

    if ! connection_exists; then
        error_message "hotspot is not installed. Run: bash raspberry-pi/hotspot.sh install"
        exit 1
    fi

    if ! output="$(sudo nmcli connection up "$HOTSPOT_CONNECTION" 2>&1)"; then
        error_message "failed to start hotspot: $output"
        exit 1
    fi

    log_message "Hotspot started. $output"
    echo "Connect to Wi-Fi: $HOTSPOT_SSID"
    echo "Control: http://$HOTSPOT_URL_IP:$WEB_PORT/control"
    echo "Tuner:   http://$HOTSPOT_URL_IP:$WEB_PORT/tuner"
}

stop_hotspot() {
    require_nmcli
    require_command readlink
    require_command systemctl
    require_command systemd-run

    if ! nmcli -t -f NAME connection show |
        grep -Fxq "$NORMAL_CONNECTION"; then
        error_message "normal Wi-Fi profile does not exist: $NORMAL_CONNECTION"
        exit 1
    fi

    script_path="$(readlink -f "$0")"
    unit_name="seba-hotspot-transition"

    if systemctl is-active --quiet "$unit_name.service" 2>/dev/null; then
        error_message "a Wi-Fi transition is already running."
        exit 1
    fi

    # Mark the transition before scheduling it so the persistent fallback
    # monitor does not reactivate the hotspot during the disconnected interval.
    sudo touch "$TRANSITION_MARKER"

    if ! output="$(
        sudo systemd-run \
            --unit="$unit_name" \
            --description="SEBA-ROBOT return to normal Wi-Fi" \
            --property=Type=oneshot \
            --collect \
            --no-block \
            --setenv="SEBA_HOTSPOT_CONNECTION=$HOTSPOT_CONNECTION" \
            --setenv="SEBA_HOTSPOT_SSID=$HOTSPOT_SSID" \
            --setenv="SEBA_HOTSPOT_IFACE=$HOTSPOT_IFACE" \
            --setenv="SEBA_HOTSPOT_IP=$HOTSPOT_IP" \
            --setenv="SEBA_HOTSPOT_CHANNEL=$HOTSPOT_CHANNEL" \
            --setenv="SEBA_WEB_PORT=$WEB_PORT" \
            --setenv="SEBA_WIFI_CONNECTION=$NORMAL_CONNECTION" \
            /usr/bin/env bash "$script_path" _down-worker 2>&1
    )"; then
        sudo rm -f "$TRANSITION_MARKER"
        error_message "failed to start Wi-Fi transition: $output"
        exit 1
    fi

    log_message "Returning to normal Wi-Fi: $NORMAL_CONNECTION"
    log_message "The hotspot connection and this SSH session may now close."
}

return_to_normal_worker() {
    require_nmcli
    require_command flock

    if [ "$(id -u)" -ne 0 ]; then
        error_message "internal transition worker must run as root."
        exit 1
    fi

    exec 9>"$TRANSITION_LOCK"

    if ! flock -n 9; then
        error_message "another Wi-Fi transition is already running."
        rm -f "$TRANSITION_MARKER"
        exit 1
    fi

    cleanup_transition() {
        rm -f "$TRANSITION_MARKER"
    }

    trap cleanup_transition EXIT

    if hotspot_active; then
        log_message "Stopping hotspot: $HOTSPOT_CONNECTION"
        nmcli connection down "$HOTSPOT_CONNECTION" >/dev/null
    fi

    log_message "Connecting to normal Wi-Fi: $NORMAL_CONNECTION"

    if output="$(nmcli connection up "$NORMAL_CONNECTION" 2>&1)"; then
        for _ in 1 2 3 4 5; do
            if normal_network_available; then
                log_message "Normal Wi-Fi connected. $NORMAL_NETWORK_REASON"
                return 0
            fi

            sleep 2
        done

        error_message "normal Wi-Fi activated but its gateway is unavailable. $NORMAL_NETWORK_REASON"
    else
        error_message "failed to activate $NORMAL_CONNECTION: $output"
    fi

    # Do not leave the Pi unreachable if the router is absent or activation
    # fails. Stop any partial normal connection and restore the hotspot.
    nmcli connection down "$NORMAL_CONNECTION" >/dev/null 2>&1 || true

    log_message "Restoring hotspot because normal Wi-Fi is unavailable."

    if ! output="$(nmcli connection up "$HOTSPOT_CONNECTION" 2>&1)"; then
        error_message "failed to restore hotspot: $output"
        return 1
    fi

    log_message "Hotspot restored: $HOTSPOT_SSID"

    return 1
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

    # A route through the SEBA hotspot is not normal network access. The
    # fallback service must only stay idle when another reachable gateway exists.
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
        exit 0
    fi

    log_message "Normal network is unavailable. Starting hotspot fallback. $NORMAL_NETWORK_REASON"
    start_hotspot
}

monitor_hotspot() {
    require_nmcli

    if ! connection_exists; then
        error_message "hotspot is not installed. Run: bash raspberry-pi/hotspot.sh install"
        exit 1
    fi

    log_message "Hotspot monitor started. wait=${FALLBACK_WAIT_SECONDS}s interval=${MONITOR_INTERVAL_SECONDS}s connection=$HOTSPOT_CONNECTION iface=$HOTSPOT_IFACE"
    sleep "$FALLBACK_WAIT_SECONDS"

    # The fallback is intentionally one-way while the hotspot is active.
    # It does not automatically abandon the hotspot when the router returns.
    # However, the monitor stays alive so fallback remains armed after a subsequent
    # explicit return to normal Wi-Fi.
    while true; do
        # The detached down worker owns wlan0 while this marker exists.
        # The monitor must not recreate the hotspot during that transition.
        if [ -e "$TRANSITION_MARKER" ]; then
            sleep "$MONITOR_INTERVAL_SECONDS"
            continue
        fi

        if hotspot_active; then
            sleep "$MONITOR_INTERVAL_SECONDS"
            continue
        fi

        if normal_network_available; then
            sleep "$MONITOR_INTERVAL_SECONDS"
            continue
        fi

        log_message "Normal network is unavailable. Starting hotspot and staying in hotspot mode. $NORMAL_NETWORK_REASON"
        start_hotspot
        sleep "$MONITOR_INTERVAL_SECONDS"
    done
}

show_status() {
    require_nmcli

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
        echo "Control: http://$HOTSPOT_URL_IP:$WEB_PORT/control"
        echo "Tuner:   http://$HOTSPOT_URL_IP:$WEB_PORT/tuner"
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
    _down-worker)
        return_to_normal_worker
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
