#!/usr/bin/env python3

import argparse
import os
import sys
from http.server import ThreadingHTTPServer
from pathlib import Path

PROJECT_DIR = Path(__file__).resolve().parents[1]
if str(PROJECT_DIR) not in sys.path:
    sys.path.insert(0, str(PROJECT_DIR))

from http_handler import TunerHandler
from seba_pi.serial_link import SerialLink


DEFAULT_BAUD = 115200
DEFAULT_SERIAL_PORT = "/dev/ttyAMA0"
DEFAULT_HTTP_PORT = 8080


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
