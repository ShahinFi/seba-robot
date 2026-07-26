# SEBA-ROBOT Raspberry Pi Web Runtime

This document describes the Raspberry Pi web server, browser interfaces, JSON API, and STM32 serial-link behavior used by SEBA-ROBOT.

The web runtime runs after the Raspberry Pi operating system, UART, network, hotspot, and services have been configured. Raspberry Pi setup is documented in [Raspberry Pi Setup](../setup/README.md).

The web runtime is not part of the real-time balance loop. Real-time sensing, state estimation, balance control, actuator control, and safety shutdown run on the STM32 firmware. The Raspberry Pi sends operator commands and displays telemetry reported by the STM32.

---

## 1. Runtime Overview

The Raspberry Pi runs one Python HTTP server. The server owns one UART connection to the STM32 and provides:

- the operator control panel
- the engineering tuner
- shared JSON endpoints for commands and telemetry
- STM32 event logs and command-status reporting

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

The browser device must be able to reach the Raspberry Pi over normal Wi-Fi, Ethernet, or the SEBA hotspot. Network setup and hotspot fallback behavior are documented in [Raspberry Pi Setup](../setup/README.md#4-hotspot-operation).

---

## 2. Running Manually

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

The installed systemd service runs the same server with the configured repository path, serial device, and web port. Service installation is documented in [Raspberry Pi Setup](../setup/README.md#5-services).

---

## 3. Web Interfaces

The server provides two browser interfaces.

### 3.1 Control Panel

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

### 3.2 Engineering Tuner

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

## 4. HTTP API

The browser applications use the shared JSON API.

### 4.1 Telemetry

```text
GET /api/telemetry
```

The telemetry response contains:

- latest parsed STM32 `TEL` values
- raw telemetry line
- stale-link status
- command queue and acknowledgement status
- STM32 text and event logs collected since the previous request

### 4.2 Commands

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

## 5. Related Documentation

- [Raspberry Pi Setup](../setup/README.md)
- [STM32 Firmware](../../firmware/stm32/README.md)
- [SEBA-ROBOT Hardware](../../hardware/README.md)
