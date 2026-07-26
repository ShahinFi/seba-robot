"""Shared HTTP routes for Raspberry Pi robot web apps."""

import json
import mimetypes
from http.server import BaseHTTPRequestHandler
from pathlib import Path
from urllib.parse import urlparse


ROOT_DIR = Path(__file__).resolve().parent
APPS_DIR = ROOT_DIR / "apps"


class RobotWebHandler(BaseHTTPRequestHandler):
    """Serve robot web apps and JSON endpoints backed by one serial link."""

    link = None

    def do_GET(self):
        path = urlparse(self.path).path

        if path in ("", "/", "/control"):
            self._send_app_file("control_panel", "index.html")
            return

        if path == "/tuner":
            self._send_app_file("tuner", "index.html")
            return

        if path.startswith("/apps/"):
            parts = path.strip("/").split("/", 2)
            if len(parts) == 3:
                self._send_app_file(parts[1], parts[2])
                return

        if path == "/api/telemetry":
            self._handle_telemetry()
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
            log_command = bool(payload.get("log", True))
            if not command:
                raise ValueError("empty command")

            response = self.link.send_command(command, log=log_command)
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

    def _handle_telemetry(self):
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
                "logs": self.link.read_logs(),
            })
        except Exception as exc:
            self._send_json(
                {"ok": False, "error": str(exc)},
                status=500
            )

    def _read_json(self):
        length = int(self.headers.get("Content-Length", "0"))
        raw = self.rfile.read(length)
        return json.loads(raw.decode("utf-8"))

    def _send_app_file(self, app_name, filename):
        """Serve one file from a known app static directory."""

        if app_name not in ("control_panel", "tuner"):
            self.send_error(404)
            return

        root = (APPS_DIR / app_name).resolve()
        path = (root / filename).resolve()

        if (
            root not in path.parents and
            path != root
        ):
            self.send_error(404)
            return

        if not path.is_file():
            self.send_error(404)
            return

        body = path.read_bytes()
        content_type = mimetypes.guess_type(path.name)[0]
        if content_type is None:
            content_type = "application/octet-stream"

        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _send_json(self, payload, status=200):
        """Send a JSON response with the given HTTP status."""

        body = json.dumps(payload).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)
