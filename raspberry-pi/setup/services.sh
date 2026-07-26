#!/usr/bin/env bash

set -eu

SCRIPT_DIR="$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)"
. "$SCRIPT_DIR/common.sh"

install_service() {
    service_name="$1"
    template="$RASPBERRY_PI_DIR/systemd/$service_name.in"
    service_file="/etc/systemd/system/$service_name"
    tmp_file="$(mktemp --suffix=.service)"

    [ -f "$template" ] || fail "Missing service template: $template"

    if [ -x "$REPO_DIR/.venv/bin/python3" ]; then
        PYTHON="$REPO_DIR/.venv/bin/python3"
    else
        PYTHON="$(command -v python3)"
    fi

    render_template "$template" "$tmp_file"
    systemd-analyze verify "$tmp_file"
    sudo install -o root -g root -m 0644 "$tmp_file" "$service_file"
    rm -f "$tmp_file"
    sudo systemctl daemon-reload
    sudo systemctl enable "$service_name"
}

install_wifi_country_service() {
    install_service seba-wifi-country.service
    sudo systemctl restart seba-wifi-country.service
}

require_root_tooling
require_command systemctl
require_command systemd-analyze

case "${1:-all}" in
    wifi-country-only)
        install_wifi_country_service
        sudo systemctl --no-pager status seba-wifi-country.service || true
        ;;
    web-only)
        install_service seba-web.service
        sudo systemctl restart seba-web.service
        sudo systemctl --no-pager status seba-web.service || true
        ;;
    hotspot-fallback-only)
        require_command nmcli

        if ! nmcli -t -f NAME connection show | grep -Fxq "$SEBA_HOTSPOT_CONNECTION"; then
            fail "Hotspot profile '$SEBA_HOTSPOT_CONNECTION' is not installed. Run network stage first."
        fi

        install_service seba-hotspot-fallback.service
        sudo systemctl restart seba-hotspot-fallback.service
        sudo systemctl --no-pager status seba-hotspot-fallback.service || true
        ;;
    all|"")
        install_wifi_country_service
        require_command nmcli

        if ! nmcli -t -f NAME connection show | grep -Fxq "$SEBA_HOTSPOT_CONNECTION"; then
            fail "Hotspot profile '$SEBA_HOTSPOT_CONNECTION' is not installed. Run network stage first."
        fi

        install_service seba-hotspot-fallback.service
        install_service seba-web.service

        sudo systemctl restart seba-hotspot-fallback.service
        sudo systemctl restart seba-web.service
        sudo systemctl --no-pager status seba-wifi-country.service || true
        sudo systemctl --no-pager status seba-hotspot-fallback.service || true
        sudo systemctl --no-pager status seba-web.service || true
        ;;
    *)
        fail "Unknown services stage argument: $1"
        ;;
esac

record_install_state
