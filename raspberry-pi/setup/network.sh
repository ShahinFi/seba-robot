#!/usr/bin/env bash

set -eu

SCRIPT_DIR="$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)"
. "$SCRIPT_DIR/common.sh"

NORMAL_WIFI_SSID="${SEBA_WIFI_SSID:-}"
NORMAL_WIFI_PASSWORD="${SEBA_WIFI_PASSWORD:-}"
NORMAL_WIFI_CONNECTION="${SEBA_WIFI_CONNECTION:-$NORMAL_WIFI_SSID}"
NORMAL_WIFI_BAND=""
NORMAL_WIFI_BSSID=""
CLOUD_INIT_DISABLE_FILE="/etc/cloud/cloud.cfg.d/99-disable-network-config.cfg"

if [ -n "${SEBA_WIFI_BAND_PROVIDED:-}" ]; then
    NORMAL_WIFI_BAND="$SEBA_WIFI_BAND"
fi

if [ -n "${SEBA_WIFI_BSSID_PROVIDED:-}" ]; then
    NORMAL_WIFI_BSSID="$SEBA_WIFI_BSSID"
fi

require_root_tooling
require_command nmcli
require_command netplan
require_command systemctl

backup_netplan() {
    backup_dir="/root/seba-network-backup/netplan-$(timestamp)"
    sudo mkdir -p "$backup_dir"
    sudo cp -a /etc/netplan/. "$backup_dir/"
    info "Netplan backup: $backup_dir" >&2
    printf '%s' "$backup_dir"
}

netplan_file_defines_seba_interface() {
    file="$1"

    sudo grep -Eq "^[[:space:]]+$SEBA_ETH_IFACE:" "$file" ||
        sudo grep -Eq "^[[:space:]]+$SEBA_HOTSPOT_IFACE:" "$file" ||
        sudo grep -Eq "^[[:space:]]+wifis:" "$file" ||
        {
            [ -n "$NORMAL_WIFI_SSID" ] &&
                sudo grep -Fq "$NORMAL_WIFI_SSID" "$file"
        }
}

handle_existing_netplan_file() {
    file="$1"
    backup_dir="$2"

    [ "$file" != "$SEBA_NETPLAN_FILE" ] || return 0
    [ -f "$file" ] || return 0

    case "$(basename "$file")" in
        90-NM-*.yaml) return 0 ;;
    esac

    netplan_file_defines_seba_interface "$file" || return 0

    # Cloud-init may own the first-boot Wi-Fi file. Replace only its network
    # content after backing it up; other conflicting Netplan files stop setup.
    case "$(basename "$file")" in
        50-cloud-init.yaml)
            tmp_file="$(mktemp)"
            cat > "$tmp_file" <<'EOF'
network:
  version: 2
EOF
            sudo install -o root -g root -m 0600 "$tmp_file" "$file"
            rm -f "$tmp_file"
            info "Replaced cloud-init Netplan network content: $file"
            ;;
        *)
            fail "$SEBA_ETH_IFACE or $SEBA_HOTSPOT_IFACE is already defined in $file.
Backup was created at $backup_dir.
Resolve the conflicting Netplan ownership before continuing."
            ;;
    esac
}

write_seba_netplan() {
    tmp_netplan="$1"

    cat > "$tmp_netplan" <<EOF
network:
  version: 2
  renderer: NetworkManager
  ethernets:
    $SEBA_ETH_IFACE:
      optional: true
      addresses:
        - $SEBA_ETH_ADDRESS
      dhcp4: false
EOF
}

restore_netplan_backup() {
    backup_dir="$1"

    sudo rm -f /etc/netplan/*.yaml
    sudo cp -a "$backup_dir"/. /etc/netplan/
    sudo netplan generate || true
}

install_cloud_init_network_disable() {
    tmp_file="$(mktemp)"
    cat > "$tmp_file" <<'EOF'
network:
  config: disabled
EOF
    sudo mkdir -p "$(dirname "$CLOUD_INIT_DISABLE_FILE")"
    write_root_file "$CLOUD_INIT_DISABLE_FILE" 0644 root "$tmp_file"
    rm -f "$tmp_file"
}

configure_normal_wifi() {
    [ -n "$NORMAL_WIFI_SSID" ] || return 0

    if nmcli -t -f NAME connection show | grep -Fxq "$NORMAL_WIFI_CONNECTION"; then
        sudo nmcli connection modify "$NORMAL_WIFI_CONNECTION" \
            connection.interface-name "$SEBA_HOTSPOT_IFACE" \
            connection.autoconnect yes \
            802-11-wireless.mode infrastructure \
            802-11-wireless.ssid "$NORMAL_WIFI_SSID" \
            802-11-wireless.powersave 2

        if [ -n "$NORMAL_WIFI_PASSWORD" ]; then
            sudo nmcli connection modify "$NORMAL_WIFI_CONNECTION" \
                802-11-wireless-security.key-mgmt wpa-psk \
                802-11-wireless-security.psk "$NORMAL_WIFI_PASSWORD"
        fi
    else
        if [ -z "$NORMAL_WIFI_PASSWORD" ]; then
            NORMAL_WIFI_PASSWORD="$(prompt_secret SEBA_WIFI_PASSWORD "Normal Wi-Fi password for $NORMAL_WIFI_SSID")"
        fi

        sudo nmcli connection add \
            type wifi \
            ifname "$SEBA_HOTSPOT_IFACE" \
            con-name "$NORMAL_WIFI_CONNECTION" \
            autoconnect yes \
            ssid "$NORMAL_WIFI_SSID" >/dev/null

        sudo nmcli connection modify "$NORMAL_WIFI_CONNECTION" \
            connection.interface-name "$SEBA_HOTSPOT_IFACE" \
            connection.autoconnect yes \
            802-11-wireless.mode infrastructure \
            802-11-wireless.ssid "$NORMAL_WIFI_SSID" \
            802-11-wireless.powersave 2 \
            802-11-wireless-security.key-mgmt wpa-psk \
            802-11-wireless-security.psk "$NORMAL_WIFI_PASSWORD"
    fi

    if [ -n "$NORMAL_WIFI_BAND" ]; then
        sudo nmcli connection modify "$NORMAL_WIFI_CONNECTION" \
            802-11-wireless.band "$NORMAL_WIFI_BAND"
    else
        # Omitted pinning values mean automatic router selection for this
        # managed normal Wi-Fi profile, even if it was pinned before.
        sudo nmcli connection modify "$NORMAL_WIFI_CONNECTION" \
            802-11-wireless.band ""
    fi

    if [ -n "$NORMAL_WIFI_BSSID" ]; then
        sudo nmcli connection modify "$NORMAL_WIFI_CONNECTION" \
            802-11-wireless.bssid "$NORMAL_WIFI_BSSID"
    else
        # Clearing the BSSID restores roaming within the same SSID instead of
        # locking this profile to one access point.
        sudo nmcli connection modify "$NORMAL_WIFI_CONNECTION" \
            802-11-wireless.bssid ""
    fi

    SEBA_WIFI_CONNECTION="$NORMAL_WIFI_CONNECTION"
    SEBA_WIFI_BAND="$NORMAL_WIFI_BAND"
    SEBA_WIFI_BSSID="$NORMAL_WIFI_BSSID"
}

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

sudo systemctl enable --now NetworkManager

backup_dir="$(backup_netplan)"

for file in /etc/netplan/*.yaml; do
    [ -e "$file" ] || continue
    handle_existing_netplan_file "$file" "$backup_dir"
done

tmp_netplan="$(mktemp)"
write_seba_netplan "$tmp_netplan"
sudo install -o root -g root -m 0600 "$tmp_netplan" "$SEBA_NETPLAN_FILE"
rm -f "$tmp_netplan"

if ! sudo netplan generate; then
    restore_netplan_backup "$backup_dir"
    fail "New Netplan configuration was invalid; previous configuration restored."
fi

install_cloud_init_network_disable

configure_normal_wifi

SEBA_WIFI_COUNTRY="$country" bash "$RASPBERRY_PI_DIR/setup/services.sh" wifi-country-only

SEBA_HOTSPOT_PASSWORD="$hotspot_password" \
SEBA_HOTSPOT_CHANNEL="$hotspot_channel" \
SEBA_HOTSPOT_IP="${SEBA_HOTSPOT_IP:-10.42.0.1/24}" \
    bash "$RASPBERRY_PI_DIR/hotspot.sh" install

info "Applying network configuration may interrupt SSH over Wi-Fi."
info "Ethernet recovery is recommended."
sudo netplan apply

info "Network setup complete."
info "Ethernet recovery: $SEBA_ETH_IFACE $eth_address"
info "Hotspot: $SEBA_HOTSPOT_SSID channel $hotspot_channel"

record_install_state
