#!/usr/bin/env bash

set -eu

SCRIPT_DIR="$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)"
SETUP_DIR="$SCRIPT_DIR/setup"

usage() {
    cat <<EOF
Usage: bash raspberry-pi/install.sh <all|preflight|packages|auth|uart|network|services|verify>

Stages:
  all       Run preflight, packages, auth, uart, network, and services.
  preflight Check the supported Raspberry Pi platform before changes.
  packages  Install operating-system packages and Python dependencies.
  auth      Create local web UI credentials.
  uart      Configure the Raspberry Pi UART for the STM32 link.
  network   Configure NetworkManager, Ethernet recovery, Wi-Fi country, and hotspot.
  services  Install and start SEBA systemd services.
  verify    Check the installed Raspberry Pi environment.

Non-interactive setup:
  Set SEBA_NONINTERACTIVE=1 and provide required values with environment variables.
  For auth, SEBA_OPERATOR_PASSWORD is required.
EOF
}

run_stage() {
    stage="$1"
    case "$stage" in
        preflight)
            bash "$SETUP_DIR/preflight.sh"
            ;;
        packages)
            bash "$SETUP_DIR/packages.sh"
            ;;
        auth)
            bash "$SETUP_DIR/auth.sh"
            ;;
        uart)
            bash "$SETUP_DIR/uart.sh"
            ;;
        network)
            bash "$SETUP_DIR/network.sh"
            ;;
        services)
            bash "$SETUP_DIR/services.sh"
            ;;
        verify)
            bash "$SETUP_DIR/verify.sh"
            ;;
        *)
            usage >&2
            exit 1
            ;;
    esac
}

case "${1:-}" in
    all)
        run_stage preflight
        run_stage packages
        run_stage auth
        run_stage uart
        run_stage network
        run_stage services
        cat <<'EOF'

Installation stages completed.

A reboot is required to activate UART boot changes and refreshed group
membership. Reboot the Raspberry Pi, reconnect, then run:

    bash raspberry-pi/install.sh verify

EOF
        ;;
    packages|auth|uart|network|services)
        run_stage preflight
        run_stage "$1"
        ;;
    preflight|verify)
        run_stage "$1"
        ;;
    -h|--help|help)
        usage
        ;;
    *)
        usage >&2
        exit 1
        ;;
esac
