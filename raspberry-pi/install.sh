#!/usr/bin/env bash

set -eu

SCRIPT_DIR="$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)"
SETUP_DIR="$SCRIPT_DIR/setup"

usage() {
    cat <<EOF
Usage: bash raspberry-pi/install.sh <all|packages|uart|network|services|verify>

Stages:
  all       Run packages, uart, network, services, and verify.
  packages  Install operating-system packages and Python dependencies.
  uart      Configure the Raspberry Pi UART for the STM32 link.
  network   Configure NetworkManager, Ethernet recovery, Wi-Fi country, and hotspot.
  services  Install and start SEBA systemd services.
  verify    Check the installed Raspberry Pi environment.

Non-interactive setup:
  Set SEBA_NONINTERACTIVE=1 and provide required values with environment variables.
EOF
}

run_stage() {
    stage="$1"
    case "$stage" in
        packages)
            bash "$SETUP_DIR/packages.sh"
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
        run_stage packages
        run_stage uart
        run_stage network
        run_stage services
        run_stage verify
        ;;
    packages|uart|network|services|verify)
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
