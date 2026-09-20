#!/usr/bin/env bash
set -euo pipefail

if [[ $# -ne 1 ]]; then
  echo "Usage: $0 <path-to-frame>" >&2
  exit 2
fi

FRAME_PATH="$1"

if [[ ! -f "$FRAME_PATH" ]]; then
  echo "Frame does not exist or is not a file: $FRAME_PATH" >&2
  exit 1
fi

exec python3 - "$FRAME_PATH" <<'PY'
import mimetypes
import os
import sys
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer


frame_path = os.path.abspath(sys.argv[1])
with open(frame_path, "rb") as frame_file:
    frame = frame_file.read()

content_type = mimetypes.guess_type(frame_path)[0] or "application/octet-stream"


class FrozenFrameHandler(BaseHTTPRequestHandler):
    def do_GET(self):  # noqa: N802 - required by BaseHTTPRequestHandler
        if self.path not in ("/", "/frame"):
            self.send_error(404)
            return

        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(frame)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(frame)

    def do_HEAD(self):  # noqa: N802 - required by BaseHTTPRequestHandler
        if self.path not in ("/", "/frame"):
            self.send_error(404)
            return

        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(frame)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()

    def log_message(self, format, *args):
        print(f"{self.address_string()} - {format % args}", file=sys.stderr)


class ReusableThreadingHTTPServer(ThreadingHTTPServer):
    allow_reuse_address = True


server = ReusableThreadingHTTPServer(("0.0.0.0", 5801), FrozenFrameHandler)
print(f"Serving frozen frame {frame_path} on 0.0.0.0:5801", flush=True)
try:
    server.serve_forever()
except KeyboardInterrupt:
    pass
finally:
    server.server_close()
PY
