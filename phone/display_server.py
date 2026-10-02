"""Runs in Termux on the phone. Serves ~/ellama (show.html + show.jpg) on port 8080 and accepts
POST /frame with a JPEG body, which atomically replaces show.jpg. The computer pushes frames
here (mcp_server/phone_display.py) and show.html, open in the phone's browser, polls for them.

  python display_server.py          # then, once:  termux-open-url http://127.0.0.1:8080/show.html

No authentication: anyone on the Wi-Fi can replace the picture (it only ever writes show.jpg,
capped at 4 MB). Fine on a home network; don't use on an untrusted one.
"""
import os
import sys
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer

ROOT = os.path.dirname(os.path.abspath(__file__))
PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 8080
MAX_BYTES = 4 * 1024 * 1024


class Handler(SimpleHTTPRequestHandler):
    def __init__(self, *a, **kw):
        super().__init__(*a, directory=ROOT, **kw)

    def end_headers(self):
        self.send_header("Cache-Control", "no-store")
        super().end_headers()

    def do_POST(self):
        if self.path != "/frame":
            self.send_error(404)
            return
        n = int(self.headers.get("Content-Length") or 0)
        if not 0 < n <= MAX_BYTES:
            self.send_error(413 if n else 400)
            return
        data = self.rfile.read(n)
        if data[:2] != b"\xff\xd8":
            self.send_error(415, "not a JPEG")
            return
        tmp = os.path.join(ROOT, ".show.tmp")
        with open(tmp, "wb") as f:
            f.write(data)
        os.replace(tmp, os.path.join(ROOT, "show.jpg"))   # atomic: the page never sees half a file
        self.send_response(204)
        self.end_headers()

    def log_message(self, *a):
        pass


if __name__ == "__main__":
    ThreadingHTTPServer(("0.0.0.0", PORT), Handler).serve_forever()
