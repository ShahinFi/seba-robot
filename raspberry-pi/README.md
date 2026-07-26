# SEBA-ROBOT Raspberry Pi Software

This document describes the Raspberry Pi software used to operate, tune, and monitor SEBA-ROBOT through the robot's local web interface.

The Raspberry Pi runs a Python web server that communicates with the STM32 over UART. The server provides the operator control panel, the engineering tuner, and shared JSON endpoints for commands, telemetry, command status, and STM32 event logs.

The Raspberry Pi software is not part of the real-time balance loop. Real-time sensing, state estimation, balance control, actuator control, and safety shutdown run on the STM32 firmware. The Raspberry Pi sends operator commands and displays the telemetry reported by the STM32.

---

## 1. Supported Platform

The supported Raspberry Pi target is:

```text
Raspberry Pi 5
Ubuntu Server 24.04 LTS 64-bit
```

Other operating systems are not covered because package names, boot files, UART naming, netplan behavior, and service management can differ.

---

## 2. Software Overview

The Raspberry Pi software has five main parts:

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

The shared server keeps one UART connection to the STM32. Both browser applications use this same connection through the same backend API.

---

## 3. Directory Structure

```text
raspberry-pi/
|-- seba_pi/
|   |-- __init__.py
|   `-- serial_link.py
|-- setup/
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
|   |-- apps/
|   |   |-- control_panel/
|   |   |   |-- app.js
|   |   |   |-- index.html
|   |   |   `-- styles.css
|   |   `-- tuner/
|   |       |-- app.js
|   |       |-- index.html
|   |       `-- styles.css
|   |-- http_handler.py
|   `-- server.py
|-- hotspot.sh
|-- install.sh
|-- requirements.txt
`-- README.md
```

| Path | Purpose |
|---|---|
| `web/server.py` | executable entry point for the Raspberry Pi web server |
| `web/http_handler.py` | HTTP routes and JSON API handlers |
| `web/apps/control_panel/` | operator control panel |
| `web/apps/tuner/` | engineering tuner |
| `seba_pi/serial_link.py` | UART connection, telemetry state, command queue, ACK handling, retries, and STM32 event logs |
| `install.sh` | public staged installer |
| `setup/` | installer stages used by `install.sh` |
| `systemd/` | systemd service templates |
| `hotspot.sh` | manual and service-driven hotspot control tool |

---

## 4. Fresh Installation

Start from a fresh Ubuntu Server image, enable SSH during imaging or first boot, give the Pi temporary Internet access, and clone the repository:

```bash
git clone https://github.com/ShahinFi/seba-robot.git
cd seba-robot
```

Run the staged installer from the repository root:

```bash
bash raspberry-pi/install.sh all
```

The `all` command installs the required packages, UART setup, network configuration, and systemd services. A reboot is required after it finishes so UART boot changes and group membership take effect. After reboot, run:

```bash
bash raspberry-pi/install.sh verify
```

The same installer can run individual stages:

```bash
bash raspberry-pi/install.sh preflight
bash raspberry-pi/install.sh packages
bash raspberry-pi/install.sh uart
bash raspberry-pi/install.sh network
bash raspberry-pi/install.sh services
bash raspberry-pi/install.sh verify
```

The `all` command stops at the first failed stage. Each stage is intended to be idempotent: running it again updates SEBA-owned configuration instead of creating duplicate services, netplan files, or NetworkManager profiles.

The installer runs preflight automatically before every modifying stage.

Interactive prompts are the default for values that are specific to the local robot or network. Non-interactive setup can be done with environment variables:

```bash
SEBA_NONINTERACTIVE=1 \
SEBA_HOTSPOT_PASSWORD='change-this-password' \
SEBA_WIFI_COUNTRY=FI \
bash raspberry-pi/install.sh all
```

In non-interactive mode, `SEBA_HOTSPOT_PASSWORD` is required. `SEBA_WIFI_COUNTRY` must match the country where the robot operates.

The installer records the resolved repository path and Linux user in:

```text
raspberry-pi/.install-state
```

Systemd services are generated from the current repository path and current user unless overridden.

---

## 5. Installer Stages

### 5.1 Packages

The package stage installs the operating-system packages used by the Raspberry Pi software:

| Package | Purpose |
|---|---|
| `git` | repository access |
| `python3`, `python3-venv`, `python3-pip` | Python runtime and project virtual environment |
| `network-manager` | NetworkManager and `nmcli` for Wi-Fi and hotspot control |
| `iw`, `rfkill` | Wi-Fi regulatory and radio diagnostics |
| `curl` | local web endpoint verification |

It also creates `.venv` in the repository root and installs:

```text
pyserial>=3.5
```

### 5.2 UART

The UART stage configures the Raspberry Pi hardware UART used for the STM32 link.

The physical wiring is:

- Raspberry Pi TX to STM32 RX
- Raspberry Pi RX to STM32 TX
- Raspberry Pi GND to STM32 GND
- 3.3 V UART logic

The detailed physical connector wiring is documented in [SEBA-ROBOT Hardware](../hardware/README.md#41-raspberry-pi-5-and-stm32g474re).

The UART stage updates:

- `/boot/firmware/config.txt`
- `/boot/firmware/cmdline.txt`

It ensures:

- `enable_uart=1`
- `console=serial0,<baud>` and `console=ttyAMA0,<baud>` are removed from the kernel command line
- the install user belongs to `dialout`

The default UART device is:

```text
/dev/ttyAMA0
```

The default baud rate is:

```text
115200
```

### 5.3 Network

The network stage configures the Raspberry Pi so a browser device can reach the Pi web server through normal Wi-Fi, Ethernet recovery, or the SEBA hotspot.

Network ownership rule:

- NetworkManager owns Wi-Fi.
- Netplan owns the renderer selection and Ethernet recovery address.
- NetworkManager owns the normal Wi-Fi and hotspot profiles.
- cloud-init network regeneration is disabled after setup.
- systemd owns the Wi-Fi country, hotspot fallback, and web services.
- Standalone `hostapd` and standalone `dnsmasq` are not used.
- `systemd-networkd` must not manage `wlan0`.

The network stage writes the SEBA-owned netplan file:

```text
/etc/netplan/99-seba-network.yaml
```

It backs up Netplan before changing it, validates the proposed Netplan configuration, and restores the backup if validation fails. It configures:

- NetworkManager as the netplan renderer
- Ethernet recovery on `eth0`
- the Wi-Fi regulatory country service
- the NetworkManager hotspot profile

Run the network stage over Ethernet when possible because `netplan apply` may interrupt Wi-Fi SSH.

Default network values:

| Item | Default |
|---|---:|
| Ethernet recovery address | `192.168.10.50/24` |
| Wi-Fi regulatory country | `FI` |
| hotspot SSID | `SEBA-ROBOT` |
| hotspot channel | `11` |
| hotspot address | `10.42.0.1/24` |

These are the tested project defaults. Override them when needed:

```bash
SEBA_WIFI_COUNTRY=US \
SEBA_HOTSPOT_CHANNEL=6 \
SEBA_ETH_ADDRESS=192.168.10.50/24 \
bash raspberry-pi/install.sh network
```

The Ethernet recovery address is intended for direct maintenance access while changing Wi-Fi modes. Configure the computer on the other end of the Ethernet cable with another address in `192.168.10.0/24`, then connect with:

| Device | Example setting |
|---|---|
| Raspberry Pi | `192.168.10.50/24` |
| maintenance computer | `192.168.10.10/24` |
| gateway | empty |

```bash
ssh <user>@192.168.10.50
```

Normal Wi-Fi credentials are not stored in the repository. To let the network stage create or update a normal Wi-Fi connection, provide them through environment variables:

```bash
SEBA_WIFI_SSID='router-name' \
SEBA_WIFI_PASSWORD='router-password' \
bash raspberry-pi/install.sh network
```

If the NetworkManager connection name is different from the SSID, set `SEBA_WIFI_CONNECTION` to the saved profile name.

If normal Wi-Fi credentials are omitted, existing NetworkManager Wi-Fi profiles are preserved.

### 5.4 Services

The service stage installs and updates the systemd services used by the Raspberry Pi software:

| Service | Purpose |
|---|---|
| `seba-web.service` | runs the Python web server |
| `seba-hotspot-fallback.service` | monitors normal network access and starts the hotspot when needed |
| `seba-wifi-country.service` | applies the Wi-Fi regulatory country before NetworkManager starts |

The service stage writes:

```text
/etc/systemd/system/seba-web.service
/etc/systemd/system/seba-hotspot-fallback.service
/etc/systemd/system/seba-wifi-country.service
```

Generated service units are checked with `systemd-analyze verify` before installation. The installer reloads systemd, enables the services, restarts them, and prints their status.

The web service uses:

- the current repository path
- the current Linux user
- `.venv/bin/python3`
- `/dev/ttyAMA0` unless overridden
- port `8080` unless overridden

Optional install-time overrides:

```bash
SEBA_SERIAL=/dev/ttyAMA0 SEBA_WEB_PORT=8080 bash raspberry-pi/install.sh services
```

| Environment variable | Purpose | Default |
|---|---|---:|
| `SEBA_SERVICE_USER` | Linux user that runs the service | current user |
| `SEBA_SERIAL` | STM32 UART device used by the service | `/dev/ttyAMA0` |
| `SEBA_WEB_PORT` | HTTP port used by the service | `8080` |

### 5.5 Verify

The verify stage checks the installed Raspberry Pi environment and exits with a nonzero status if a required condition is not met.

Run:

```bash
bash raspberry-pi/install.sh verify
```

The final line is one of:

```text
SEBA Raspberry Pi setup: PASS
SEBA Raspberry Pi setup: FAIL
```

On failure, the script lists only the failed checks.

---

## 6. Network and Hotspot Operation

In normal use, connect the Raspberry Pi to an existing Wi-Fi or Ethernet network using NetworkManager. The phone, tablet, or computer used for control must be connected to the same local network.

The browser does not communicate with the STM32 directly. It opens the Raspberry Pi web server over HTTP. The Raspberry Pi receives those requests, sends the corresponding UART commands to the STM32, and serves the latest STM32 telemetry back to the browser.

The web server listens on all Raspberry Pi network interfaces by default:

```text
0.0.0.0:8080
```

This means the pages can be opened from another device on the same local network by using the Raspberry Pi hostname or IP address.

### 6.1 Hotspot Profile

The hotspot profile is named:

```text
seba-hotspot
```

The profile uses:

- AP mode on `wlan0`
- 2.4 GHz band
- WPA2/RSN with CCMP
- PMF optional
- IPv4 shared mode
- IPv6 disabled
- Wi-Fi power saving disabled
- autoconnect disabled

Default hotspot settings:

| Item | Value |
|---|---:|
| NetworkManager connection | `seba-hotspot` |
| Wi-Fi interface | `wlan0` |
| Wi-Fi SSID | `SEBA-ROBOT` |
| Raspberry Pi hotspot address | `10.42.0.1` |
| default hotspot channel | `11` |
| control panel URL | `http://10.42.0.1:8080/control` |
| tuner URL | `http://10.42.0.1:8080/tuner` |

### 6.2 Manual Hotspot Control

`hotspot.sh` is the manual and service-driven hotspot control tool:

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

The hotspot is not enabled automatically by `hotspot.sh install`. Start it manually with:

```bash
bash raspberry-pi/hotspot.sh up
```

### 6.3 Fallback Behavior

The hotspot fallback service waits after boot, monitors normal network access, and starts the `SEBA-ROBOT` hotspot if normal network access is unavailable or lost later.

Once the fallback service starts the hotspot, it leaves the Raspberry Pi in hotspot mode. It does not automatically switch back to normal Wi-Fi when the previous network returns.

Return to normal Wi-Fi is manual:

```bash
bash raspberry-pi/hotspot.sh down
sudo nmcli connection up "<normal-wifi-profile>"
sudo systemctl restart seba-hotspot-fallback.service
```

The hotspot fallback service can become inactive after it successfully starts the hotspot. That is normal for the one-way fallback policy.

---

## 7. Running Manually

Run the web server from the repository root:

```bash
.venv/bin/python3 raspberry-pi/web/server.py --serial /dev/ttyAMA0 --port 8080
```

The same settings can be provided through environment variables:

```bash
SEBA_SERIAL=/dev/ttyAMA0 SEBA_BAUD=115200 SEBA_WEB_HOST=0.0.0.0 SEBA_WEB_PORT=8080 .venv/bin/python3 raspberry-pi/web/server.py
```

Runtime settings:

| Command-line option | Environment variable | Default |
|---|---|---:|
| `--serial` | `SEBA_SERIAL` | `/dev/ttyAMA0` |
| `--baud` | `SEBA_BAUD` | `115200` |
| `--host` | `SEBA_WEB_HOST` | `0.0.0.0` |
| `--port` | `SEBA_WEB_PORT` | `8080` |

---

## 8. Web Interfaces

The server provides two browser interfaces.

### 8.1 Control Panel

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

### 8.2 Engineering Tuner

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

## 9. HTTP API

The browser applications use the shared JSON API.

### 9.1 Telemetry

```text
GET /api/telemetry
```

The telemetry response contains:

- latest parsed STM32 `TEL` values
- raw telemetry line
- stale-link status
- command queue and acknowledgement status
- STM32 text and event logs collected since the previous request

### 9.2 Commands

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

## 10. Service Commands and Logs

Useful service commands:

```bash
sudo systemctl status seba-web.service
sudo systemctl status seba-hotspot-fallback.service
sudo systemctl status seba-wifi-country.service
```

View logs:

```bash
sudo journalctl -u seba-web.service -f
sudo journalctl -u seba-hotspot-fallback.service -f
sudo journalctl -u seba-wifi-country.service -f
```

---

## 11. Verification and Troubleshooting

Run the full verification stage:

```bash
bash raspberry-pi/install.sh verify
```

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

Check that the Raspberry Pi serial console is not using the robot UART. For this robot connection, the kernel command line should not contain `console=serial0,<baud>` or `console=ttyAMA0,<baud>`.

```bash
cat /boot/firmware/cmdline.txt
```

Check that the Raspberry Pi firmware configuration enables the UART pins:

```bash
grep enable_uart /boot/firmware/config.txt
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

Final manual acceptance test:

1. Boot with router Wi-Fi available and confirm normal Wi-Fi connects.
2. Disconnect normal Wi-Fi and confirm the `SEBA-ROBOT` hotspot starts.
3. Connect a phone or laptop to the hotspot and confirm it receives a `10.42.0.x` address.
4. Open `http://10.42.0.1:8080/control`.
5. Return manually to normal Wi-Fi if needed.
6. Reboot and confirm the settings persist.

If the webpage shows stale telemetry or serial errors, check:

- the STM32 is powered and running
- Raspberry Pi TX is connected to STM32 RX
- STM32 TX is connected to Raspberry Pi RX
- Raspberry Pi GND and STM32 GND share the robot common ground
- the selected serial device matches the actual UART device
- no other process is holding the same serial device
