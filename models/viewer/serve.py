"""
Team Blueprint — 3D Viewer Local Web Server
Runs a lightweight HTTP server on port 8000 with CORS and auto-opens the browser.
"""

import http.server
import socketserver
import webbrowser
import os
import sys

PORT = 8000
DIRECTORY = os.path.normpath(os.path.join(os.path.dirname(__file__), "..", ".."))

class CORSRequestHandler(http.server.SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=DIRECTORY, **kwargs)

    def end_headers(self):
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Cache-Control', 'no-store, must-revalidate')
        super().end_headers()

def main():
    os.chdir(DIRECTORY)
    url = f"http://localhost:{PORT}/models/viewer/"
    print("=" * 60)
    print("  TEAM BLUEPRINT — 3D ROBOT VIEWER")
    print(f"  Serving directory: {DIRECTORY}")
    print(f"  URL: {url}")
    print("  Press Ctrl+C to stop the server.")
    print("=" * 60)

    # Open default browser
    webbrowser.open(url)

    # Allow port reuse
    socketserver.TCPServer.allow_reuse_address = True
    with socketserver.TCPServer(("", PORT), CORSRequestHandler) as httpd:
        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            print("\nShutting down 3D viewer server...")
            sys.exit(0)

if __name__ == "__main__":
    main()
