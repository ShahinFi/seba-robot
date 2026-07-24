#!/usr/bin/env python3

"""Run the SEBA-ROBOT Raspberry Pi web server."""

import argparse
import os
from http.server import ThreadingHTTPServer

from http_handler import RobotWebHandler
from seba_pi.serial_link import SerialLink


DEFAULT_BAUD = 115200
DEFAULT_SERIAL_PORT = "/dev/ttyAMA0"
DEFAULT_HTTP_PORT = 8080


def main():
    """Parse runtime settings and start the shared robot HTTP server."""

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
        default=os.environ.get("SEBA_WEB_HOST", "0.0.0.0")
    )
    parser.add_argument(
        "--port",
        type=int,
        default=int(os.environ.get("SEBA_WEB_PORT", DEFAULT_HTTP_PORT))
    )
    args = parser.parse_args()

    RobotWebHandler.link = SerialLink(args.serial, args.baud)
    server = ThreadingHTTPServer((args.host, args.port), RobotWebHandler)

    print(
        f"SEBA web server listening on http://{args.host}:{args.port} "
        f"using {args.serial} @ {args.baud}"
    )
    server.serve_forever()


if __name__ == "__main__":
    main()
