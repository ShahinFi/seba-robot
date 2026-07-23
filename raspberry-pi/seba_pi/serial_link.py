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
