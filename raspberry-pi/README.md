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
|   `-- seba-robot.service.in
|-- http_handler.py
|-- install_service.sh
|-- requirements.txt
`-- server.py
```

- `server.py` is the executable entry point for the Raspberry Pi web server.
- `http_handler.py` contains the HTTP routes and JSON API handlers.
- `seba_pi/serial_link.py` maintains the UART connection, telemetry state, command queue, acknowledgements, retries, and STM32 event logs.
- `apps/control_panel/` contains the operator control panel.
- `apps/tuner/` contains the engineering tuner for live controller and actuator parameters.
- `systemd/seba-robot.service.in` is the systemd service template.
- `install_service.sh` installs and enables the systemd service on the Raspberry Pi.

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

The Raspberry Pi user must have permission to access the serial device. On Ubuntu, the user should be in the `dialout` group.

```bash
sudo usermod -aG dialout "$USER"
```

Log out and log back in after changing group membership.

---

## 4. Python Environment

Install the Python virtual-environment package if it is not already available:

```bash
sudo apt install python3-venv
```

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
- emergency actuator stop
- STM32 reset hold button
- joystick drive and turn commands
- drive-speed and turn-speed limits
- robot state and actuator telemetry
- fault banner
- STM32 event log

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
- actuator stop
- motion commands
- balance gain scale
- maximum wheel torque
- actuator current-loop gains
- actuator current, PWM, integral, and torque-constant limits
- RSLQR gain matrix entries
- telemetry and STM32 event logs

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
bash raspberry-pi/install_service.sh
```

The installer writes:

```text
/etc/systemd/system/seba-robot.service
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
SEBA_SERIAL=/dev/ttyAMA0 SEBA_WEB_PORT=8080 bash raspberry-pi/install_service.sh
```

| Environment variable | Purpose | Default |
|---|---|---:|
| `SEBA_SERVICE_USER` | Linux user that runs the service | current user |
| `SEBA_SERIAL` | STM32 UART device used by the service | `/dev/ttyAMA0` |
| `SEBA_WEB_PORT` | HTTP port used by the service | `8080` |

Useful service commands:

```bash
sudo systemctl status seba-robot.service
sudo systemctl restart seba-robot.service
sudo systemctl stop seba-robot.service
sudo systemctl disable seba-robot.service
```

View server logs:

```bash
sudo journalctl -u seba-robot.service -f
```

---

## 9. Serial Troubleshooting

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

The firmware configuration should enable the UART pins:

```text
enable_uart=1
```

If the webpage shows stale telemetry or serial errors, check:

- the STM32 is powered and running
- Raspberry Pi TX is connected to STM32 RX
- STM32 TX is connected to Raspberry Pi RX
- Raspberry Pi GND and STM32 GND share the robot common ground
- the selected serial device matches the actual UART device
- no other process is holding the same serial device
