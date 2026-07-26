# SEBA-ROBOT Raspberry Pi Software

This document describes the Raspberry Pi software used to operate, tune, and monitor SEBA-ROBOT through the robot's local web interface.

The Raspberry Pi runs a Python web server that communicates with the STM32 over UART. The server provides the operator control panel, the engineering tuner, and shared JSON endpoints for commands, telemetry, command status, and STM32 event logs.

The Raspberry Pi software is not part of the real-time balance loop. Real-time sensing, state estimation, balance control, actuator control, and safety shutdown run on the STM32 firmware. The Raspberry Pi sends operator commands and displays the telemetry reported by the STM32.

---

## 1. Software Overview

The Raspberry Pi software has three main parts:

- one shared Python HTTP server
- one reusable STM32 serial-link module
- two browser applications

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

The shared server keeps one UART connection to the STM32. Both web applications use this same connection through the same backend API.

---

## 2. Directory Structure

```text
raspberry-pi/
|-- apps/
|   |-- control_panel/
|   |   |-- app.js
|   |   |-- index.html
|   |   `-- styles.css
|   `-- tuner/
|       |-- app.js
|       |-- index.html
|       `-- styles.css
|-- seba_pi/
|   |-- __init__.py
|   `-- serial_link.py
|-- systemd/
|   |-- seba-hotspot-fallback.service.in
|   `-- seba-web.service.in
|-- hotspot.sh
|-- http_handler.py
|-- install_hotspot_fallback.sh
|-- install_web_service.sh
|-- requirements.txt
`-- server.py
```

- `server.py` is the executable entry point for the Raspberry Pi web server.
- `http_handler.py` contains the HTTP routes and JSON API handlers.
- `seba_pi/serial_link.py` maintains the UART connection, telemetry state, command queue, acknowledgements, retries, and STM32 event logs.
- `apps/control_panel/` contains the operator control panel.
- `apps/tuner/` contains the engineering tuner for live controller and actuator parameters.
- `systemd/seba-web.service.in` is the web-server systemd service template.
- `systemd/seba-hotspot-fallback.service.in` is the optional hotspot monitor service template.
- `hotspot.sh` installs, starts, stops, and checks the optional Wi-Fi hotspot.
- `install_hotspot_fallback.sh` installs the optional hotspot monitor service.
- `install_web_service.sh` installs and enables the web-server systemd service on the Raspberry Pi.

---

## 3. Hardware Interface

The Raspberry Pi communicates with the STM32 through UART.

The default serial device is:

```text
/dev/ttyAMA0
```

The default baud rate is:

```text
115200
```

The physical UART wiring is documented in [SEBA-ROBOT Hardware](../hardware/README.md#41-raspberry-pi-5-and-stm32g474re).

---

## 4. Raspberry Pi System Setup

The Raspberry Pi needs four setup items before the web server can operate reliably:

- required operating-system packages
- a Python virtual environment with the server dependency installed
- access to the STM32 UART device
- network access so a browser device can reach the Pi web server

### 4.1 Operating-system packages

Install the required base packages:

```bash
sudo apt update
sudo apt install -y python3-venv python3-pip
```

Required package purpose:

| Package | Purpose |
|---|---|
| `python3-venv` | creates the project virtual environment |
| `python3-pip` | installs Python dependencies inside the virtual environment |

The optional hotspot mode requires NetworkManager:

```bash
sudo apt install -y network-manager
sudo systemctl enable --now NetworkManager
nmcli device status
```

| Package | Purpose |
|---|---|
| `network-manager` | provides `nmcli` and manages the optional Wi-Fi hotspot connection |

For hotspot mode, the Wi-Fi interface must be managed by NetworkManager. On Ubuntu images that use `systemd-networkd`, configure netplan to use NetworkManager:

```bash
sudo tee /etc/netplan/99-seba-networkmanager.yaml >/dev/null <<'EOF'
network:
  version: 2
  renderer: NetworkManager
EOF

sudo chmod 600 /etc/netplan/99-seba-networkmanager.yaml
sudo netplan generate
sudo netplan apply
```

`netplan apply` can briefly interrupt SSH because it restarts network configuration.

Check the result:

```bash
nmcli device status
networkctl status wlan0 --no-pager
```

Expected result:

- `nmcli` shows `wlan0` as `connected` or `disconnected`, not `unavailable`
- `networkctl` does not show a `systemd-networkd` network file managing `wlan0`

### 4.2 Python environment

Create and activate the virtual environment from the repository root:

```bash
python3 -m venv .venv
. .venv/bin/activate
```

Install the Raspberry Pi server dependencies:

```bash
pip install -r raspberry-pi/requirements.txt
```

The dependency list is:

```text
pyserial>=3.5
```

### 4.3 UART and serial permissions

The Raspberry Pi talks to the STM32 through the hardware UART. The default UART device used by this project is:

```text
/dev/ttyAMA0
```

The Linux user that runs the server must be able to open this device without `sudo`. On Ubuntu, add the user to the `dialout` group:

```bash
sudo usermod -aG dialout "$USER"
```

Log out and log back in after changing group membership.

The Raspberry Pi firmware configuration must enable the UART pins:

```text
enable_uart=1
```

The Linux serial console must not use the robot UART. For this robot connection, the kernel command line should not contain:

```text
console=serial0,115200
```

### 4.4 Network access

The Raspberry Pi web server needs the Pi to be reachable over a network.

In normal use, connect the Raspberry Pi to an existing Wi-Fi or Ethernet network using the operating-system network tools. The phone, tablet, or computer used for control must be connected to the same local network.

The browser does not communicate with the STM32 directly. It opens the Raspberry Pi web server over HTTP. The Raspberry Pi receives those requests, sends the corresponding UART commands to the STM32, and serves the latest STM32 telemetry back to the browser.

The web server listens on all Raspberry Pi network interfaces by default:

```text
0.0.0.0:8080
```

This means the pages can be opened from another device on the same local network by using the Raspberry Pi hostname or IP address.

A Raspberry Pi can also be configured as a Wi-Fi hotspot/access point. This repository provides an optional NetworkManager setup script for that mode. The hotspot is system network configuration; it is not part of the Python web server.

Manual hotspot control:

```bash
bash raspberry-pi/hotspot.sh install
bash raspberry-pi/hotspot.sh up
bash raspberry-pi/hotspot.sh down
bash raspberry-pi/hotspot.sh status
```

The `install` command creates or updates the hotspot profile and sets its password. The password must be 8 to 63 characters. For non-interactive setup, provide it as an environment variable:

```bash
SEBA_HOTSPOT_PASSWORD='change-this-password' bash raspberry-pi/hotspot.sh install
```

Default hotspot settings:

| Item | Value |
|---|---:|
| NetworkManager connection | `seba-hotspot` |
| Wi-Fi interface | `wlan0` |
| Wi-Fi SSID | `SEBA-ROBOT` |
| Raspberry Pi hotspot address | `10.42.0.1` |
| control panel URL | `http://10.42.0.1:8080/control` |
| tuner URL | `http://10.42.0.1:8080/tuner` |

The hotspot is not enabled automatically by `hotspot.sh install`. Start it manually with:

```bash
bash raspberry-pi/hotspot.sh up
```

Optional automatic hotspot fallback can be installed separately after `hotspot.sh install` has created the NetworkManager hotspot connection. The service waits after boot, monitors normal network access, and starts the `SEBA-ROBOT` hotspot if normal network access is unavailable or lost later.

```bash
bash raspberry-pi/install_hotspot_fallback.sh
```

Once the fallback service starts the hotspot, it leaves the Raspberry Pi in hotspot mode. It does not automatically switch back to normal Wi-Fi when the previous network returns.

---

## 5. Running Manually

Run the web server from the repository root:

```bash
python3 raspberry-pi/server.py --serial /dev/ttyAMA0 --port 8080
```

The same settings can be provided through environment variables:

```bash
SEBA_SERIAL=/dev/ttyAMA0 SEBA_BAUD=115200 SEBA_WEB_HOST=0.0.0.0 SEBA_WEB_PORT=8080 python3 raspberry-pi/server.py
```

Runtime settings:

| Command-line option | Environment variable | Default |
|---|---|---:|
| `--serial` | `SEBA_SERIAL` | `/dev/ttyAMA0` |
| `--baud` | `SEBA_BAUD` | `115200` |
| `--host` | `SEBA_WEB_HOST` | `0.0.0.0` |
| `--port` | `SEBA_WEB_PORT` | `8080` |

---

## 6. Web Interfaces

The server provides two browser interfaces.

### 6.1 Control Panel

The control panel is the operator interface:

```text
http://seba-pi:8080/
http://seba-pi:8080/control
```

If hostname resolution is not available, use the Raspberry Pi IP address:

```text
http://<raspberry-pi-ip>:8080/
http://<raspberry-pi-ip>:8080/control
```

The control panel provides:

- balance start and stop
- emergency stop button
- STM32 reset hold button
- joystick drive and turn commands
- drive-speed and turn-speed limits
- stop motion command that zeros the requested drive and turn speeds
- robot state and actuator telemetry
- fault banner
- STM32 event log

On desktop and portrait mobile screens, the control panel uses a stacked responsive layout. On phone-sized landscape screens, it switches to a controller-style layout: the joystick fills the left side, while balance start and stop, STM32 reset, speed limits, and stop motion stay on the right side.

Joystick commands are sent repeatedly while the joystick is held away from center. When the joystick is released, the command returns to zero.

### 6.2 Engineering Tuner

The tuner is the engineering interface:

```text
http://seba-pi:8080/tuner
```

If hostname resolution is not available, use:

```text
http://<raspberry-pi-ip>:8080/tuner
```

The tuner provides live access to:

- balance start and stop
- STM32 hold-to-reset
- joystick motion commands with maximum drive and turn speed limits
- direct forward-velocity and yaw-rate commands
- grouped actuator-loop configuration
- grouped balance-loop configuration, including gain scale, torque limit, and RSLQR gain matrix
- grouped telemetry display
- command log and STM32 event log

The tuner header shows the serial connection state and the current balance mode.

The tuner uses a wide three-column layout on desktop screens. Below the wide-screen breakpoint, sections stack vertically in this order: run and motion controls, actuator settings, balance settings, telemetry, and logs.

The actuator settings are applied with one `actuator config ...` command. The balance-loop settings are applied with one `balance config ...` command. This keeps each apply action atomic from the web interface instead of sending one command per field.

The tuner is used for development and parameter adjustment. The control panel is used for normal robot operation.

---

## 7. HTTP API

The browser applications use the shared JSON API.

### Telemetry

```text
GET /api/telemetry
```

The telemetry response contains:

- latest parsed STM32 `TEL` values
- raw telemetry line
- stale-link status
- command queue and acknowledgement status
- STM32 text and event logs collected since the previous request

### Commands

```text
POST /api/command
```

The command body is JSON:

```json
{
  "command": "balance start",
  "log": true
}
```

The `log` field controls whether the command appears in the browser command log. Repeated joystick keepalive commands use `false` so they do not fill the log.

Commands are sent to the STM32 using the acknowledged command protocol:

```text
CMD <id> <command text>
```

The STM32 responds with:

```text
ACK <id> OK
ACK <id> ERROR
```

The serial link retries each command until an acknowledgement is received or the retry limit is reached.

The following urgent commands replace queued commands so they are not delayed behind lower-priority requests:

- `balance stop`
- `actuator stop`
- `system reset`

---

## 8. Autostart Service

The Raspberry Pi server can be installed as a systemd service.

Create the Python environment and install the dependencies before installing the service.

Run from the repository root:

```bash
bash raspberry-pi/install_web_service.sh
```

The installer writes:

```text
/etc/systemd/system/seba-web.service
```

It then reloads systemd, enables the service, restarts it, and prints the service status.

The service uses:

- the current repository path
- the current Linux user
- `.venv/bin/python3` if it exists
- `/dev/ttyAMA0` unless overridden
- port `8080` unless overridden

Optional install-time overrides:

```bash
SEBA_SERIAL=/dev/ttyAMA0 SEBA_WEB_PORT=8080 bash raspberry-pi/install_web_service.sh
```

| Environment variable | Purpose | Default |
|---|---|---:|
| `SEBA_SERVICE_USER` | Linux user that runs the service | current user |
| `SEBA_SERIAL` | STM32 UART device used by the service | `/dev/ttyAMA0` |
| `SEBA_WEB_PORT` | HTTP port used by the service | `8080` |

Useful service commands:

```bash
sudo systemctl status seba-web.service
sudo systemctl restart seba-web.service
sudo systemctl stop seba-web.service
sudo systemctl disable seba-web.service
```

View server logs:

```bash
sudo journalctl -u seba-web.service -f
```

---

## 9. Verification and Troubleshooting

Check that the UART device exists:

```bash
ls -l /dev/ttyAMA0
```

Check that the user has serial permission:

```bash
groups
```

The group list should include:

```text
dialout
```

Check that the Raspberry Pi serial console is not using the robot UART. For this robot connection, the kernel command line should not contain `console=serial0,115200`.

```bash
cat /boot/firmware/cmdline.txt
```

Check that the Raspberry Pi firmware configuration enables the UART pins:

```bash
grep enable_uart /boot/firmware/config.txt
```

Check that the web service is running:

```bash
sudo systemctl status seba-web.service
```

Check that the web server responds locally on the Pi:

```bash
curl -I http://127.0.0.1:8080/control
```

Check hotspot state and fallback service state:

```bash
bash raspberry-pi/hotspot.sh status
sudo systemctl status seba-hotspot-fallback.service --no-pager
```

If the webpage shows stale telemetry or serial errors, check:

- the STM32 is powered and running
- Raspberry Pi TX is connected to STM32 RX
- STM32 TX is connected to Raspberry Pi RX
- Raspberry Pi GND and STM32 GND share the robot common ground
- the selected serial device matches the actual UART device
- no other process is holding the same serial device
