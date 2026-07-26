#!/usr/bin/env bash

set -eu

COMMON_DIR="$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
RASPBERRY_PI_DIR="$(dirname "$COMMON_DIR")"
REPO_DIR="$(dirname "$RASPBERRY_PI_DIR")"

save_explicit_value() {
    name="$1"
    if [ "${!name+x}" ]; then
        printf -v "_EXPLICIT_$name" '%s' "${!name}"
        printf -v "_HAS_EXPLICIT_$name" '%s' 1
        printf -v "${name}_PROVIDED" '%s' 1
    else
        printf -v "${name}_PROVIDED" '%s' ''
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
    SEBA_SERVICE_USER \
    SEBA_SERIAL \
    SEBA_WEB_PORT \
    SEBA_HOTSPOT_CONNECTION \
    SEBA_HOTSPOT_SSID \
    SEBA_HOTSPOT_IFACE \
    SEBA_HOTSPOT_IP \
    SEBA_HOTSPOT_CHANNEL \
    SEBA_HOTSPOT_PASSWORD \
    SEBA_WIFI_COUNTRY \
    SEBA_WIFI_CONNECTION \
    SEBA_WIFI_SSID \
    SEBA_WIFI_PASSWORD \
    SEBA_ETH_IFACE \
    SEBA_ETH_ADDRESS \
    SEBA_NONINTERACTIVE
do
    save_explicit_value "$name"
done

SEBA_INSTALL_STATE_FILE="${SEBA_INSTALL_STATE_FILE:-$RASPBERRY_PI_DIR/.install-state}"
if [ -f "$SEBA_INSTALL_STATE_FILE" ]; then
    . "$SEBA_INSTALL_STATE_FILE"
fi

for name in \
    SEBA_SERVICE_USER \
    SEBA_SERIAL \
    SEBA_WEB_PORT \
    SEBA_HOTSPOT_CONNECTION \
    SEBA_HOTSPOT_SSID \
    SEBA_HOTSPOT_IFACE \
    SEBA_HOTSPOT_IP \
    SEBA_HOTSPOT_CHANNEL \
    SEBA_HOTSPOT_PASSWORD \
    SEBA_WIFI_COUNTRY \
    SEBA_WIFI_CONNECTION \
    SEBA_WIFI_SSID \
    SEBA_WIFI_PASSWORD \
    SEBA_ETH_IFACE \
    SEBA_ETH_ADDRESS \
    SEBA_NONINTERACTIVE
do
    restore_explicit_value "$name"
done

SEBA_INSTALL_USER="${SEBA_SERVICE_USER:-${SEBA_INSTALL_USER:-$(id -un)}}"
SEBA_NONINTERACTIVE="${SEBA_NONINTERACTIVE:-0}"
SEBA_SERIAL="${SEBA_SERIAL:-/dev/ttyAMA0}"
SEBA_WEB_PORT="${SEBA_WEB_PORT:-8080}"
SEBA_HOTSPOT_CONNECTION="${SEBA_HOTSPOT_CONNECTION:-seba-hotspot}"
SEBA_HOTSPOT_SSID="${SEBA_HOTSPOT_SSID:-SEBA-ROBOT}"
SEBA_HOTSPOT_IFACE="${SEBA_HOTSPOT_IFACE:-wlan0}"
SEBA_HOTSPOT_IP="${SEBA_HOTSPOT_IP:-10.42.0.1/24}"
SEBA_HOTSPOT_CHANNEL="${SEBA_HOTSPOT_CHANNEL:-11}"
SEBA_WIFI_COUNTRY="${SEBA_WIFI_COUNTRY:-FI}"
SEBA_WIFI_CONNECTION="${SEBA_WIFI_CONNECTION:-}"
SEBA_ETH_IFACE="${SEBA_ETH_IFACE:-eth0}"
SEBA_ETH_ADDRESS="${SEBA_ETH_ADDRESS:-192.168.10.50/24}"
SEBA_NETPLAN_FILE="${SEBA_NETPLAN_FILE:-/etc/netplan/99-seba-network.yaml}"

timestamp() {
    date '+%Y%m%d-%H%M%S'
}

info() {
    printf '%s\n' "$*"
}

fail() {
    printf 'ERROR: %s\n' "$*" >&2
    exit 1
}

require_command() {
    command -v "$1" >/dev/null 2>&1 || fail "$1 is required."
}

require_root_tooling() {
    require_command sudo
}

is_noninteractive() {
    [ "$SEBA_NONINTERACTIVE" = "1" ]
}

prompt_value() {
    variable_name="$1"
    prompt_text="$2"
    default_value="$3"
    current_value="$(eval "printf '%s' \"\${$variable_name:-}\"")"
    provided="$(eval "printf '%s' \"\${${variable_name}_PROVIDED:-}\"")"

    if [ -n "$provided" ]; then
        printf '%s' "$current_value"
        return
    fi

    if is_noninteractive; then
        printf '%s' "$default_value"
        return
    fi

    printf '%s [%s]: ' "$prompt_text" "$default_value" >&2
    read -r value
    if [ -z "$value" ]; then
        value="$default_value"
    fi

    printf '%s' "$value"
}

prompt_secret() {
    variable_name="$1"
    prompt_text="$2"
    current_value="$(eval "printf '%s' \"\${$variable_name:-}\"")"

    if [ -n "$current_value" ]; then
        printf '%s' "$current_value"
        return
    fi

    if is_noninteractive; then
        fail "$variable_name must be set in non-interactive mode."
    fi

    printf '%s: ' "$prompt_text" >&2
    trap 'stty echo; printf "\n" >&2; exit 1' INT TERM
    stty -echo
    read -r value
    stty echo
    trap - INT TERM
    printf '\n' >&2

    printf '%s' "$value"
}

backup_file() {
    path="$1"
    if [ -f "$path" ]; then
        backup="${path}.seba-backup-$(timestamp)"
        sudo cp "$path" "$backup"
        info "Backup: $backup"
    fi
}

backup_file_if_changed() {
    path="$1"
    candidate="$2"

    if [ -f "$path" ] && cmp -s "$path" "$candidate"; then
        return
    fi

    backup_file "$path"
}

write_root_file() {
    destination="$1"
    mode="$2"
    owner="$3"
    source_file="$4"

    backup_file_if_changed "$destination" "$source_file"
    sudo install -o "$owner" -g "$owner" -m "$mode" "$source_file" "$destination"
}

record_install_state() {
    tmp_file="$(mktemp "${SEBA_INSTALL_STATE_FILE}.tmp.XXXXXX")"
    {
        printf 'SEBA_INSTALL_USER=%q\n' "$SEBA_INSTALL_USER"
        printf 'SEBA_REPO_DIR=%q\n' "$REPO_DIR"
        printf 'SEBA_SERIAL=%q\n' "$SEBA_SERIAL"
        printf 'SEBA_WEB_PORT=%q\n' "$SEBA_WEB_PORT"
        printf 'SEBA_HOTSPOT_CONNECTION=%q\n' "$SEBA_HOTSPOT_CONNECTION"
        printf 'SEBA_HOTSPOT_SSID=%q\n' "$SEBA_HOTSPOT_SSID"
        printf 'SEBA_HOTSPOT_IFACE=%q\n' "$SEBA_HOTSPOT_IFACE"
        printf 'SEBA_HOTSPOT_IP=%q\n' "$SEBA_HOTSPOT_IP"
        printf 'SEBA_HOTSPOT_CHANNEL=%q\n' "$SEBA_HOTSPOT_CHANNEL"
        printf 'SEBA_WIFI_COUNTRY=%q\n' "$SEBA_WIFI_COUNTRY"
        printf 'SEBA_WIFI_CONNECTION=%q\n' "$SEBA_WIFI_CONNECTION"
        printf 'SEBA_ETH_IFACE=%q\n' "$SEBA_ETH_IFACE"
        printf 'SEBA_ETH_ADDRESS=%q\n' "$SEBA_ETH_ADDRESS"
    } > "$tmp_file"
    chmod 600 "$tmp_file"
    mv "$tmp_file" "$SEBA_INSTALL_STATE_FILE"
}

render_template() {
    template="$1"
    output="$2"
    sed \
        -e "s|@USER@|$SEBA_INSTALL_USER|g" \
        -e "s|@REPO_DIR@|$REPO_DIR|g" \
        -e "s|@SERIAL_PORT@|$SEBA_SERIAL|g" \
        -e "s|@WEB_PORT@|$SEBA_WEB_PORT|g" \
        -e "s|@PYTHON@|$PYTHON|g" \
        -e "s|@WIFI_COUNTRY@|$SEBA_WIFI_COUNTRY|g" \
        "$template" > "$output"
}
