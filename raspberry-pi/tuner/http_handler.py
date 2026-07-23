"""HTTP routes for the Raspberry Pi tuner app."""

import json
import mimetypes
from http.server import BaseHTTPRequestHandler
from pathlib import Path
from urllib.parse import urlparse


STATIC_DIR = Path(__file__).with_name("static")


class TunerHandler(BaseHTTPRequestHandler):
    """Serve tuner assets and JSON endpoints backed by a link object."""

    link = None

    def do_GET(self):
        path = urlparse(self.path).path

        if path == "/":
            self._send_static_file("index.html")
            return

        if path in ("/styles.css", "/app.js"):
            self._send_static_file(path.lstrip("/"))
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

    def _send_static_file(self, filename):
        """Serve a file from the tuner static asset directory."""

        path = STATIC_DIR / filename
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
