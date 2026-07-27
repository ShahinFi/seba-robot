# SEBA-ROBOT Raspberry Pi Platform

This directory contains the Raspberry Pi web runtime and setup tooling used to operate, tune, and monitor SEBA-ROBOT through local web interfaces.

The Raspberry Pi hosts the operator control panel and engineering tuner, communicates with the STM32 over UART, and provides browser access over normal Wi-Fi, Ethernet, or the SEBA hotspot. It does not run the real-time balance loop. Real-time sensing, state estimation, balance control, actuator control, and safety shutdown run on the STM32 firmware.

---

## 1. Platform Parts

The Raspberry Pi platform has five main parts:

- one shared Python HTTP server
- one reusable STM32 serial-link module
- two browser applications
- a staged Raspberry Pi installer
- NetworkManager-based hotspot and fallback tooling

```text
+----------------------------+
| Raspberry Pi web browser   |
+-------------+--------------+
              |
              | HTTP
              v
+-------------+--------------+
| Python web server          |
| /control, /tuner, /api/*   |
+-------------+--------------+
              |
              | UART command and telemetry link
              v
+-------------+--------------+
| STM32G474RE firmware       |
| real-time robot control    |
+----------------------------+
```

---

## 2. Directory Structure

```text
raspberry-pi/
|-- seba_pi/
|   |-- auth.py
|   |-- auth_hash.py
|   |-- __init__.py
|   `-- serial_link.py
|-- setup/
|   |-- README.md
|   |-- auth.sh
|   |-- common.sh
|   |-- network.sh
|   |-- packages.sh
|   |-- preflight.sh
|   |-- services.sh
|   |-- uart.sh
|   `-- verify.sh
|-- systemd/
|   |-- seba-hotspot-fallback.service.in
|   |-- seba-web.service.in
|   `-- seba-wifi-country.service.in
|-- web/
|   |-- README.md
|   |-- apps/
|   |   |-- control_panel/
|   |   `-- tuner/
|   |-- http_handler.py
|   `-- server.py
|-- hotspot.sh
|-- install.sh
|-- local_config.example.env
|-- requirements.txt
`-- README.md
```

| Path | Purpose |
|---|---|
| `web/` | Python web server, login-gated control panel, tuner, JSON API, and web-runtime documentation |
| `seba_pi/` | local web authentication, UART connection, telemetry state, command queue, ACK handling, retries, and STM32 event logs |
| `install.sh` | public staged installer for Raspberry Pi setup |
| `setup/` | setup-stage scripts and setup documentation |
| `systemd/` | service templates installed by the setup tooling |
| `hotspot.sh` | manual and service-driven hotspot control tool |

---

## 3. Main Workflows

Fresh Raspberry Pi setup:

```bash
bash raspberry-pi/install.sh all
```

After the installer finishes, reboot the Raspberry Pi and run:

```bash
bash raspberry-pi/install.sh verify
```

Manual web-server run from the repository root:

```bash
.venv/bin/python3 raspberry-pi/web/server.py --serial /dev/ttyAMA0 --port 8080
```

Default web interfaces:

```text
http://seba-pi:8080/control
http://seba-pi:8080/tuner
```

Default hotspot web interfaces:

```text
http://10.42.0.1:8080/control
http://10.42.0.1:8080/tuner
```

---

## 4. Documentation

### Raspberry Pi Setup

[**Raspberry Pi Setup**](setup/README.md)

Fresh Ubuntu setup, installer stages, packages, UART, NetworkManager, Netplan, Ethernet recovery, hotspot fallback, systemd services, verification, and troubleshooting.

### Raspberry Pi Web Runtime

[**Raspberry Pi Web Runtime**](web/README.md)

Python web server, local login, operator control panel, engineering tuner, JSON API, STM32 command path, telemetry, and event logs.

### Related Project Documentation

- [STM32 Firmware](../firmware/stm32/README.md)
- [SEBA-ROBOT Hardware](../hardware/README.md)
