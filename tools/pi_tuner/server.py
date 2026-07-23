#!/usr/bin/env python3

import argparse
import json
import os
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse

try:
    import serial
except ImportError:
    serial = None


DEFAULT_BAUD = 115200
DEFAULT_SERIAL_PORT = "/dev/ttyAMA0"
DEFAULT_HTTP_PORT = 8080
SERIAL_TIMEOUT_S = 0.02
COMMAND_ACK_RETRY_S = 0.15
COMMAND_MAX_ATTEMPTS = 20


INDEX_HTML = r"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>SEBA Tuner</title>
  <style>
    :root {
      color-scheme: dark;
      --bg: #101317;
      --panel: #181d23;
      --line: #2b333d;
      --text: #edf2f7;
      --muted: #9aa7b4;
      --accent: #47b881;
      --danger: #ff5a5f;
      --warn: #e3a72f;
    }

    * {
      box-sizing: border-box;
    }

    body {
      margin: 0;
      font-family: system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
      background: var(--bg);
      color: var(--text);
    }

    header {
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 16px;
      padding: 16px 20px;
      border-bottom: 1px solid var(--line);
      background: #14191f;
      position: sticky;
      top: 0;
      z-index: 2;
    }

    h1 {
      margin: 0;
      font-size: 20px;
      font-weight: 650;
    }

    main {
      display: grid;
      grid-template-columns: minmax(320px, 460px) minmax(360px, 1fr);
      gap: 16px;
      padding: 16px;
    }

    section {
      background: var(--panel);
      border: 1px solid var(--line);
      border-radius: 8px;
      padding: 14px;
    }

    h2 {
      margin: 0 0 12px;
      font-size: 15px;
      font-weight: 650;
      color: #d8e0e8;
    }

    .grid {
      display: grid;
      grid-template-columns: repeat(2, minmax(0, 1fr));
      gap: 10px;
    }

    .full {
      grid-column: 1 / -1;
    }

    label {
      display: grid;
      gap: 6px;
      color: var(--muted);
      font-size: 12px;
    }

    input {
      width: 100%;
      border: 1px solid var(--line);
      border-radius: 6px;
      background: #0f1318;
      color: var(--text);
      padding: 8px;
      font: inherit;
    }

    input[type="range"] {
      padding: 0;
    }

    button {
      border: 1px solid var(--line);
      border-radius: 6px;
      background: #222a33;
      color: var(--text);
      padding: 9px 11px;
      font: inherit;
      cursor: pointer;
    }

    button:hover {
      border-color: #4a5968;
    }

    button.primary {
      background: #1f5f46;
      border-color: #297956;
    }

    button.danger {
      background: #6b2528;
      border-color: #8a3034;
    }

    .toolbar {
      display: flex;
      flex-wrap: wrap;
      gap: 8px;
    }

    .status {
      color: var(--muted);
      font-size: 13px;
    }

    .status.ok {
      color: var(--accent);
    }

    .status.bad {
      color: var(--danger);
    }

    .readout {
      display: grid;
      grid-template-columns: repeat(4, minmax(0, 1fr));
      gap: 8px;
    }

    .metric {
      border: 1px solid var(--line);
      border-radius: 6px;
      padding: 9px;
      background: #12171d;
      min-height: 58px;
    }

    .metric span {
      display: block;
      color: var(--muted);
      font-size: 11px;
      margin-bottom: 5px;
    }

    .metric strong {
      font-size: 18px;
      font-weight: 650;
    }

    .matrix {
      display: grid;
      grid-template-columns: 52px repeat(6, minmax(64px, 1fr));
      gap: 6px;
      align-items: center;
    }

    .matrix .head {
      color: var(--muted);
      font-size: 11px;
      text-align: center;
    }

    .log {
      height: 190px;
      overflow: auto;
      background: #0c1014;
      border: 1px solid var(--line);
      border-radius: 6px;
      padding: 8px;
      color: #c9d4df;
      font-family: ui-monospace, SFMono-Regular, Consolas, monospace;
      font-size: 12px;
      white-space: pre-wrap;
    }

    @media (max-width: 980px) {
      main {
        grid-template-columns: 1fr;
      }

      .readout {
        grid-template-columns: repeat(2, minmax(0, 1fr));
      }
    }
  </style>
</head>
<body>
  <header>
    <h1>SEBA-ROBOT Live Tuner</h1>
    <div id="connection" class="status">connecting</div>
  </header>

  <main>
    <div>
      <section>
        <h2>Run Control</h2>
        <div class="toolbar">
          <button class="primary" onclick="sendCommand('balance start')">Start Balance</button>
          <button onclick="sendCommand('balance stop')">Stop Balance</button>
          <button class="danger" onclick="sendCommand('actuator stop')">Stop Actuator</button>
          <button class="danger" onclick="resetStm32()">Reset STM32</button>
        </div>
      </section>

      <section>
        <h2>Motion Command</h2>
        <div class="grid">
          <label>Forward velocity [m/s]
            <input id="v_cmd" type="number" min="-2" max="2" step="0.01" value="0">
          </label>
          <label>Yaw rate [rad/s]
            <input id="yaw_cmd" type="number" min="-6" max="6" step="0.01" value="0">
          </label>
          <button class="full" onclick="applyMotionCommand()">Apply Motion Command</button>
        </div>
      </section>

      <section>
        <h2>Balance Loop</h2>
        <div class="grid">
          <label>Gain scale [%]
            <input id="gain_scale" type="number" min="0" max="10000" step="1" value="200">
          </label>
          <label>Max torque [mNm]
            <input id="max_torque" type="number" min="0" max="10000" step="10" value="5000">
          </label>
          <button onclick="sendValue('balance gain-scale', 'gain_scale')">Apply Gain Scale</button>
          <button onclick="sendValue('balance max-torque', 'max_torque')">Apply Max Torque</button>
        </div>
      </section>

      <section>
        <h2>Actuator Loop</h2>
        <div class="grid">
          <label>Kp [mV/A]
            <input id="act_kp" type="number" min="0" max="100000" step="100" value="4000">
          </label>
          <label>Ki [mV/(A*s)]
            <input id="act_ki" type="number" min="0" max="100000" step="100" value="0">
          </label>
          <label>Max current [mA]
            <input id="act_max_current" type="number" min="0" max="10000" step="50" value="5000">
          </label>
          <label>Max PWM [%]
            <input id="act_max_pwm" type="number" min="0" max="100" step="1" value="100">
          </label>
          <label>Integral limit [mV]
            <input id="act_integral" type="number" min="0" max="20000" step="100" value="3000">
          </label>
          <label>Torque constant [mNm/A]
            <input id="act_ktw" type="number" min="1" max="10000" step="10" value="1000">
          </label>
          <button onclick="sendValue('actuator kp', 'act_kp')">Apply Kp</button>
          <button onclick="sendValue('actuator ki', 'act_ki')">Apply Ki</button>
          <button onclick="sendValue('actuator max-current', 'act_max_current')">Apply Max Current</button>
          <button onclick="sendValue('actuator max-pwm', 'act_max_pwm')">Apply Max PWM</button>
          <button onclick="sendValue('actuator integral-limit', 'act_integral')">Apply Integral</button>
          <button onclick="sendValue('actuator torque-constant', 'act_ktw')">Apply Torque Constant</button>
        </div>
      </section>
    </div>

    <div>
      <section>
        <h2>Telemetry</h2>
        <div class="readout" id="readout"></div>
      </section>

      <section>
        <h2>RSLQR Gain Matrix</h2>
        <div class="matrix" id="gain_matrix"></div>
        <div style="height:10px"></div>
        <button onclick="applyAllGains()">Apply All Gains</button>
      </section>

      <section>
        <h2>Command Log</h2>
        <div id="log" class="log"></div>
      </section>
    </div>
  </main>

  <script>
    const defaultGains = [
      [-2.0, -1.0, -2.8, -10.0, -1.2, -0.16],
      [-2.0,  1.0, -2.8, -10.0, -1.2,  0.16]
    ];

    const metrics = [
      "valid", "balance", "fault", "fall",
      "theta", "theta_dot", "theta_ddot", "v",
      "left_T", "right_T", "left_dT", "right_dT",
      "left_ref", "right_ref", "left_meas", "right_meas",
      "left_pwm", "right_pwm", "gain", "max_T"
    ];

    function log(text) {
      const node = document.getElementById("log");
      const time = new Date().toLocaleTimeString();
      node.textContent = `[${time}] ${text}\n` + node.textContent;
    }

    function buildReadout() {
      const readout = document.getElementById("readout");
      readout.innerHTML = "";
      for (const key of metrics) {
        const box = document.createElement("div");
        box.className = "metric";
        box.innerHTML = `<span>${key}</span><strong id="m_${key}">-</strong>`;
        readout.appendChild(box);
      }
    }

    function buildGainMatrix() {
      const matrix = document.getElementById("gain_matrix");
      matrix.textContent = "";
      matrix.appendChild(document.createElement("div"));

      for (let col = 0; col < 6; col++) {
        const heading = document.createElement("div");
        heading.className = "head";
        heading.textContent = `Z${col}`;
        matrix.appendChild(heading);
      }

      for (let row = 0; row < 2; row++) {
        const label = document.createElement("div");
        label.className = "head";
        label.textContent = row === 0 ? "left" : "right";
        matrix.appendChild(label);

        for (let col = 0; col < 6; col++) {
          const input = document.createElement("input");
          input.id = `gain_${row}_${col}`;
          input.type = "number";
          input.step = "0.001";
          input.value = defaultGains[row][col];
          matrix.appendChild(input);
        }
      }
    }

    async function api(path, options) {
      const response = await fetch(path, options);
      const data = await response.json();
      if (!response.ok || data.ok === false) {
        throw new Error(data.error || "request failed");
      }
      return data;
    }

    async function sendCommand(command) {
      const data = await api("/api/command", {
        method: "POST",
        headers: {"Content-Type": "application/json"},
        body: JSON.stringify({command})
      });
      log(`${command} -> ${data.response || "OK"}`);
      return data;
    }

    function sendValue(prefix, inputId) {
      const value = document.getElementById(inputId).value;
      return sendCommand(`${prefix} ${value}`);
    }

    function applyMotionCommand() {
      const v = document.getElementById("v_cmd").value;
      const yaw = document.getElementById("yaw_cmd").value;
      return sendCommand(`balance command ${v} ${yaw}`);
    }

    function resetStm32() {
      return sendCommand("system reset");
    }

    async function applyAllGains() {
      for (let row = 0; row < 2; row++) {
        const side = row === 0 ? "left" : "right";
        for (let col = 0; col < 6; col++) {
          const value = document.getElementById(`gain_${row}_${col}`).value;
          await sendCommand(`balance gain ${side} ${col} ${value}`);
        }
      }
    }

    let telemetryPollActive = false;
    let lastAck = "";

    async function pollTelemetry() {
      if (telemetryPollActive) {
        return;
      }

      telemetryPollActive = true;
      const status = document.getElementById("connection");

      try {
        const data = await api("/api/telemetry");
        status.textContent = "connected";
        status.className = "status ok";
        if (
          data.command &&
          data.command.ack &&
          data.command.ack !== lastAck
        ) {
          lastAck = data.command.ack;
          log(`STM ${data.command.ack}`);
        }

        for (const key of metrics) {
          const node = document.getElementById(`m_${key}`);
          if (
            node &&
            Object.prototype.hasOwnProperty.call(data.telemetry, key) &&
            data.telemetry[key] !== null
          ) {
            node.textContent = data.telemetry[key];
          }
        }
      } catch (error) {
        status.textContent = error.message;
        status.className = "status bad";
      } finally {
        telemetryPollActive = false;
      }
    }

    buildReadout();
    buildGainMatrix();
    pollTelemetry();
    setInterval(pollTelemetry, 300);
  </script>
</body>
</html>
"""


class SerialLink:
    def __init__(self, port, baud):
        self.port = port
        self.baud = baud
        self._serial = None
        self._condition = threading.Condition()
        self._commands = deque()
        self._inflight_command = None
        self._next_command_id = 1
        self._telemetry = None
        self._telemetry_raw = ""
        self._last_telemetry_s = 0.0
        self._last_ack = ""
        self._error = "starting"
        self._stop = threading.Event()
        self._thread = threading.Thread(
            target=self._run,
            daemon=True,
        )
        self._thread.start()

    def _open(self):
        if serial is None:
            raise RuntimeError("pyserial is not installed")

        if self._serial is None or not self._serial.is_open:
            self._serial = serial.Serial(
                self.port,
                self.baud,
                timeout=SERIAL_TIMEOUT_S,
                write_timeout=SERIAL_TIMEOUT_S,
            )
            self._serial.reset_input_buffer()

        return self._serial

    def send_command(
        self,
        command,
    ):
        text = command.strip()
        if not text:
            raise ValueError("empty command")

        with self._condition:
            command_id = self._next_command_id
            self._next_command_id += 1
            if self._next_command_id > 2147483647:
                self._next_command_id = 1

            command = {
                "id": command_id,
                "text": text,
                "attempts": 0,
                "next_send": 0.0,
            }

            if text in ("balance stop", "actuator stop", "system reset"):
                self._commands.clear()
                self._inflight_command = command
            else:
                self._commands.append(command)

            self._condition.notify()

        return f"queued id={command_id}"

    def snapshot(self):
        age = time.monotonic() - self._last_telemetry_s
        if self._telemetry is None:
            return None, "", self._error

        if age > 1.0:
            return self._telemetry, self._telemetry_raw, self._error

        return self._telemetry, self._telemetry_raw, None

    def command_status(self):
        with self._condition:
            inflight_id = (
                self._inflight_command["id"]
                if self._inflight_command is not None
                else None
            )

            return {
                "ack": self._last_ack,
                "inflight": inflight_id,
                "queued": len(self._commands),
            }

    def _run(self):
        while not self._stop.is_set():
            try:
                self._read_stream_line()
                self._service_command()
            except Exception as exc:
                self._error = str(exc)
                self._close()
                self._wait_for_command(0.1)

    def _service_command(self):
        now = time.monotonic()

        with self._condition:
            if self._inflight_command is None:
                self._inflight_command = self._pop_command_locked()

            command = self._inflight_command

        if command is None:
            return

        if command["next_send"] > now:
            return

        if command["attempts"] >= COMMAND_MAX_ATTEMPTS:
            self._error = f"command {command['id']} not acknowledged"
            self._inflight_command = None
            return

        self._write_line(
            f"CMD {command['id']} {command['text']}"
        )

        command["attempts"] += 1
        command["next_send"] = now + COMMAND_ACK_RETRY_S

    def _pop_command_locked(self):
        if not self._commands:
            return None

        return self._commands.popleft()

    def _wait_for_command(self, timeout):
        with self._condition:
            if not self._commands:
                self._condition.wait(timeout)

    def _write_line(self, line):
        device = self._open()
        device.write((line + "\n").encode("ascii"))
        device.flush()

    def _read_stream_line(self):
        device = self._open()
        raw = device.readline()
        if not raw:
            return

        line = raw.decode("utf-8", errors="replace").strip()
        if not line:
            return

        if line.startswith("TEL "):
            self._telemetry = parse_telemetry(line)
            self._telemetry_raw = line
            self._last_telemetry_s = time.monotonic()
            self._error = None
            return

        if line.startswith("ACK "):
            self._handle_ack(line)

    def _handle_ack(self, line):
        parts = line.split()
        if len(parts) < 3:
            return

        try:
            command_id = int(parts[1])
        except ValueError:
            return

        if parts[2] not in ("OK", "ERROR"):
            return

        with self._condition:
            if (
                self._inflight_command is not None and
                self._inflight_command["id"] == command_id
            ):
                self._inflight_command = None
                self._last_ack = line

                if parts[2] == "OK":
                    self._error = None
                else:
                    self._error = line

    def _close(self):
        if self._serial is not None:
            try:
                self._serial.close()
            except Exception:
                pass
            self._serial = None


def parse_telemetry(line):
    if not line.startswith("TEL "):
        raise ValueError("telemetry response missing TEL prefix")

    values = {}
    for item in line[4:].split():
        if "=" not in item:
            continue

        key, value = item.split("=", 1)
        try:
            if "." in value or value.startswith("-"):
                values[key] = float(value)
            else:
                values[key] = int(value)
        except ValueError:
            values[key] = value

    return values


class TunerHandler(BaseHTTPRequestHandler):
    link = None

    def do_GET(self):
        path = urlparse(self.path).path

        if path == "/":
            self._send_html(INDEX_HTML)
            return

        if path == "/api/telemetry":
            try:
                telemetry, line, error = self.link.snapshot()
                if telemetry is None:
                    raise RuntimeError(error or "telemetry unavailable")

                self._send_json({
                    "ok": True,
                    "telemetry": telemetry,
                    "raw": line,
                    "stale": error is not None,
                    "error": error,
                    "command": self.link.command_status(),
                })
            except Exception as exc:
                self._send_json(
                    {"ok": False, "error": str(exc)},
                    status=500
                )
            return

        self.send_error(404)

    def do_POST(self):
        path = urlparse(self.path).path

        if path != "/api/command":
            self.send_error(404)
            return

        try:
            payload = self._read_json()
            command = str(payload["command"]).strip()
            if not command:
                raise ValueError("empty command")

            response = self.link.send_command(command)
            self._send_json({
                "ok": not response.startswith("ERROR:"),
                "response": response,
            })
        except Exception as exc:
            self._send_json(
                {"ok": False, "error": str(exc)},
                status=500
            )

    def log_message(self, format, *args):
        return

    def _read_json(self):
        length = int(self.headers.get("Content-Length", "0"))
        raw = self.rfile.read(length)
        return json.loads(raw.decode("utf-8"))

    def _send_html(self, html):
        body = html.encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _send_json(self, payload, status=200):
        body = json.dumps(payload).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--serial",
        default=os.environ.get("SEBA_SERIAL", DEFAULT_SERIAL_PORT)
    )
    parser.add_argument(
        "--baud",
        type=int,
        default=int(os.environ.get("SEBA_BAUD", DEFAULT_BAUD))
    )
    parser.add_argument(
        "--host",
        default=os.environ.get("SEBA_TUNER_HOST", "0.0.0.0")
    )
    parser.add_argument(
        "--port",
        type=int,
        default=int(os.environ.get("SEBA_TUNER_PORT", DEFAULT_HTTP_PORT))
    )
    args = parser.parse_args()

    TunerHandler.link = SerialLink(args.serial, args.baud)
    server = ThreadingHTTPServer((args.host, args.port), TunerHandler)

    print(
        f"SEBA tuner serving on http://{args.host}:{args.port} "
        f"using {args.serial} @ {args.baud}"
    )
    server.serve_forever()


if __name__ == "__main__":
    main()
