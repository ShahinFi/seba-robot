#!/usr/bin/env bash

set -eu

SCRIPT_DIR="$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)"
. "$SCRIPT_DIR/common.sh"

prepared_services=""
installed_services=""

prepare_service() {
    service_name="$1"
    template="$RASPBERRY_PI_DIR/systemd/$service_name.in"
    tmp_file="$(mktemp --suffix=.service)"

    [ -f "$template" ] ||
        fail "Missing service template: $template"

    case "$service_name" in
        seba-web.service)
            [ -x "$REPO_DIR/.venv/bin/python3" ] ||
                fail "Python virtual environment missing. Run the packages stage first."
            [ -f "$RASPBERRY_PI_DIR/local_config.env" ] ||
                fail "Web auth config missing. Run the auth stage first."
            id "$SEBA_INSTALL_USER" >/dev/null 2>&1 ||
                fail "Service user does not exist: $SEBA_INSTALL_USER"
            PYTHON="$REPO_DIR/.venv/bin/python3"
            ;;
        *)
            PYTHON=""
            ;;
    esac

    render_template "$template" "$tmp_file"
    systemd-analyze verify "$tmp_file"
    prepared_services="${prepared_services}${service_name}|${tmp_file}
"
}

install_prepared_services() {
    printf '%s' "$prepared_services" |
        while IFS='|' read -r service_name tmp_file; do
            [ -n "$service_name" ] || continue
            sudo install -o root -g root -m 0644 \
                "$tmp_file" "/etc/systemd/system/$service_name"
            rm -f "$tmp_file"
    done

    installed_services="$prepared_services"
    installed_services="$(printf '%s' "$installed_services" | cut -d '|' -f 1)"
    installed_services="${installed_services}
"
}

cleanup_prepared_services() {
    printf '%s' "$prepared_services" |
        while IFS='|' read -r service_name tmp_file; do
            [ -n "$service_name" ] || continue
            [ ! -f "$tmp_file" ] || rm -f "$tmp_file"
        done
}

finish_services() {
    [ -n "$prepared_services" ] || return 0

    install_prepared_services
    sudo systemctl daemon-reload

    [ -n "$installed_services" ] || return 0

    printf '%s' "$installed_services" |
        while IFS= read -r service_name; do
            [ -n "$service_name" ] || continue
            sudo systemctl enable "$service_name"
        done

    printf '%s' "$installed_services" |
        while IFS= read -r service_name; do
            [ -n "$service_name" ] || continue
            sudo systemctl restart "$service_name"
        done
}

show_statuses() {
    printf '%s' "$installed_services" |
        while IFS= read -r service_name; do
            [ -n "$service_name" ] || continue
            sudo systemctl --no-pager status "$service_name" || true
        done
}

require_root_tooling
require_command systemctl
require_command systemd-analyze

trap cleanup_prepared_services EXIT

case "${1:-all}" in
    wifi-country-only)
        prepare_service seba-wifi-country.service
        ;;
    web-only)
        prepare_service seba-web.service
        ;;
    hotspot-fallback-only)
        require_command nmcli

        if ! nmcli -t -f NAME connection show | grep -Fxq "$SEBA_HOTSPOT_CONNECTION"; then
            fail "Hotspot profile '$SEBA_HOTSPOT_CONNECTION' is not installed. Run network stage first."
        fi

        prepare_service seba-hotspot-fallback.service
        ;;
    all|"")
        prepare_service seba-wifi-country.service
        require_command nmcli

        if ! nmcli -t -f NAME connection show | grep -Fxq "$SEBA_HOTSPOT_CONNECTION"; then
            fail "Hotspot profile '$SEBA_HOTSPOT_CONNECTION' is not installed. Run network stage first."
        fi

        prepare_service seba-hotspot-fallback.service
        prepare_service seba-web.service
        ;;
    *)
        fail "Unknown services stage argument: $1"
        ;;
esac

finish_services
trap - EXIT
show_statuses
record_install_state
