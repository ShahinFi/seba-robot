"""Reusable STM32 serial link for Raspberry Pi applications."""

import threading
import time
from collections import deque

try:
    import serial
except ImportError:
    serial = None


SERIAL_TIMEOUT_S = 0.02
COMMAND_ACK_RETRY_S = 0.15
COMMAND_MAX_ATTEMPTS = 20
RX_BUFFER_LIMIT = 4096


class SerialLink:
    """Maintain telemetry, queued commands, retries, and ACK state."""

    def __init__(self, port, baud):
        self.port = port
        self.baud = baud
        self._serial = None
        self._rx_buffer = bytearray()
        self._condition = threading.Condition()
        self._commands = deque()
        self._inflight_command = None
        self._next_command_id = 1
        self._telemetry = None
        self._telemetry_raw = ""
        self._last_telemetry_s = 0.0
        self._last_ack = ""
        self._logs = deque(maxlen=80)
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
        log=True,
    ):
        """Queue one STM32 command and return the assigned command ID."""

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
                "log": log,
                "attempts": 0,
                "next_send": 0.0,
            }

            # Stop and reset commands replace queued motion/tuning commands so
            # safety actions are not delayed behind stale browser requests.
            if text in ("balance stop", "actuator stop", "system reset"):
                self._commands.clear()
                self._inflight_command = command
            else:
                self._commands.append(command)

            self._condition.notify()

        return f"queued id={command_id}"

    def snapshot(self):
        """Return the latest telemetry snapshot and any link error."""

        age = time.monotonic() - self._last_telemetry_s
        if self._telemetry is None:
            return None, "", self._error

        if age > 1.0:
            return self._telemetry, self._telemetry_raw, self._error

        return self._telemetry, self._telemetry_raw, None

    def command_status(self):
        """Return the current command queue and last ACK state."""

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

    def read_logs(self):
        """Return and clear STM32 text lines collected since the last call."""

        with self._condition:
            logs = list(self._logs)
            self._logs.clear()

        return logs

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

        # The STM32 treats repeated command IDs as idempotent, so retries can
        # recover from lost ACK lines without executing the command twice.
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
        read_size = max(1, device.in_waiting)
        raw = device.read(read_size)
        if not raw:
            return

        self._rx_buffer.extend(raw)

        if len(self._rx_buffer) > RX_BUFFER_LIMIT:
            # A missing newline or line noise must not grow memory without
            # bound; dropping the partial line keeps the link recoverable.
            self._rx_buffer.clear()
            self._error = "serial receive line too long"
            return

        while b"\n" in self._rx_buffer:
            line_raw, _, remaining = self._rx_buffer.partition(b"\n")
            self._rx_buffer = bytearray(remaining)

            self._handle_stream_line(line_raw)

    def _handle_stream_line(self, raw):
        line = raw.decode("utf-8", errors="replace").strip()
        if not line:
            return

        if line.startswith("TEL "):
            self._telemetry = parse_telemetry(line)
            self._telemetry_raw = line
            self._last_telemetry_s = time.monotonic()
            self._error = None
            return

        # ACK and EVT lines are side channels. They update command/log state
        # without replacing the latest telemetry snapshot used by the UI.
        if line.startswith("ACK "):
            self._handle_ack(line)
            return

        if line.startswith("EVT "):
            with self._condition:
                self._logs.append(line)
            return

        with self._condition:
            if (
                self._inflight_command is None or
                self._inflight_command["log"]
            ):
                self._logs.append(line)

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
                should_log = self._inflight_command["log"]
                self._inflight_command = None
                if should_log:
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
            self._rx_buffer.clear()


def parse_telemetry(line):
    """Parse one TEL line from the STM32 into typed key/value data."""

    if not line.startswith("TEL "):
        raise ValueError("telemetry response missing TEL prefix")

    values = {}
    for item in line[4:].split():
        if "=" not in item:
            continue

        key, value = item.split("=", 1)
        try:
            if value.lstrip("-").isdigit():
                values[key] = int(value)
            else:
                values[key] = float(value)
        except ValueError:
            values[key] = value

    return values
