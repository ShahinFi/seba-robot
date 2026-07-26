#!/usr/bin/env bash

set -eu

SCRIPT_DIR="$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)"
. "$SCRIPT_DIR/common.sh"

NORMAL_WIFI_SSID="${SEBA_WIFI_SSID:-}"
NORMAL_WIFI_PASSWORD="${SEBA_WIFI_PASSWORD:-}"
NORMAL_WIFI_CONNECTION="${SEBA_WIFI_CONNECTION:-$NORMAL_WIFI_SSID}"

require_root_tooling
require_command nmcli
require_command netplan
require_command systemctl

country="$(prompt_value SEBA_WIFI_COUNTRY "Wi-Fi regulatory country" "$SEBA_WIFI_COUNTRY")"
hotspot_channel="$(prompt_value SEBA_HOTSPOT_CHANNEL "Hotspot channel" "$SEBA_HOTSPOT_CHANNEL")"
eth_address="$(prompt_value SEBA_ETH_ADDRESS "Ethernet recovery address" "$SEBA_ETH_ADDRESS")"
hotspot_password="$(prompt_secret SEBA_HOTSPOT_PASSWORD "Hotspot password for $SEBA_HOTSPOT_SSID")"

SEBA_WIFI_COUNTRY="$country"
SEBA_HOTSPOT_CHANNEL="$hotspot_channel"
SEBA_ETH_ADDRESS="$eth_address"

if [ "${#hotspot_password}" -lt 8 ] || [ "${#hotspot_password}" -gt 63 ]; then
    fail "Hotspot password must be 8 to 63 characters."
fi

tmp_netplan="$(mktemp)"
cat > "$tmp_netplan" <<EOF
network:
  version: 2
  renderer: NetworkManager
  ethernets:
    $SEBA_ETH_IFACE:
      addresses:
        - $eth_address
      optional: true
EOF

sudo netplan generate --debug >/dev/null 2>&1 || {
    rm -f "$tmp_netplan"
    fail "Existing netplan configuration is invalid before SEBA changes."
}

write_root_file "$SEBA_NETPLAN_FILE" 0600 root "$tmp_netplan"
rm -f "$tmp_netplan"

sudo netplan generate
sudo systemctl enable --now NetworkManager

if [ -n "$NORMAL_WIFI_SSID" ]; then
    if nmcli -t -f NAME connection show | grep -Fxq "$NORMAL_WIFI_CONNECTION"; then
        sudo nmcli connection modify "$NORMAL_WIFI_CONNECTION" \
            connection.autoconnect yes \
            802-11-wireless.powersave 2
    else
        if [ -z "$NORMAL_WIFI_PASSWORD" ]; then
            NORMAL_WIFI_PASSWORD="$(prompt_secret SEBA_WIFI_PASSWORD "Normal Wi-Fi password for $NORMAL_WIFI_SSID")"
        fi
        sudo nmcli device wifi connect "$NORMAL_WIFI_SSID" password "$NORMAL_WIFI_PASSWORD" ifname "$SEBA_HOTSPOT_IFACE"
        NORMAL_WIFI_CONNECTION="$NORMAL_WIFI_SSID"
        sudo nmcli connection modify "$NORMAL_WIFI_CONNECTION" \
            connection.autoconnect yes \
            802-11-wireless.powersave 2
    fi
    SEBA_WIFI_CONNECTION="$NORMAL_WIFI_CONNECTION"
fi

SEBA_WIFI_COUNTRY="$country" bash "$RASPBERRY_PI_DIR/setup/services.sh" wifi-country-only

SEBA_HOTSPOT_PASSWORD="$hotspot_password" \
SEBA_HOTSPOT_CHANNEL="$hotspot_channel" \
SEBA_HOTSPOT_IP="${SEBA_HOTSPOT_IP:-10.42.0.1/24}" \
    bash "$RASPBERRY_PI_DIR/hotspot.sh" install

sudo netplan apply

info "Network setup complete."
info "Ethernet recovery: $SEBA_ETH_IFACE $eth_address"
info "Hotspot: $SEBA_HOTSPOT_SSID channel $hotspot_channel"
info "If SSH disconnects during netplan apply, reconnect through Wi-Fi or Ethernet recovery."

record_install_state
