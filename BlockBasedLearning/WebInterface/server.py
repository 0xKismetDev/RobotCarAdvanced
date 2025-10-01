#!/usr/bin/env python3
import http.server
import socketserver
import os
import sys
import webbrowser
from threading import Timer

PORT = 8080
HOST = "localhost"

class CustomHTTPRequestHandler(http.server.SimpleHTTPRequestHandler):
    def end_headers(self):
        # Add CORS headers for local development
        self.send_header('Cache-Control', 'no-store, no-cache, must-revalidate')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')
        return super(CustomHTTPRequestHandler, self).end_headers()

    def log_message(self, format, *args):
        # Custom logging format
        sys.stderr.write("%s - %s\n" % (self.address_string(), format % args))

def open_browser():
    """Open the default web browser after server starts"""
    webbrowser.open(f'http://{HOST}:{PORT}')

def main():
    # Change to the directory containing the web files
    web_dir = os.path.dirname(os.path.abspath(__file__))
    os.chdir(web_dir)

    print("=" * 60)
    print("   Blockly Robot Car - Local Server")
    print("=" * 60)
    print()
    print(f"Starting server on http://{HOST}:{PORT}")
    print()
    print("Instructions:")
    print("1. This server will open your browser automatically")
    print("2. Make sure your ESP32 is powered on")
    print("3. Connect to the ESP32's WiFi network:")
    print("   Network: RobotCar-Blockly")
    print("   Password: 12345678")
    print("4. In the web interface, enter ESP32 IP: 192.168.4.1")
    print("5. Click Connect to start programming!")
    print()
    print("Press Ctrl+C to stop the server")
    print("-" * 60)

    # Create server
    with socketserver.TCPServer(("", PORT), CustomHTTPRequestHandler) as httpd:
        # Open browser after 1 second
        timer = Timer(1, open_browser)
        timer.start()

        try:
            # Start serving
            httpd.serve_forever()
        except KeyboardInterrupt:
            print("\n\nServer stopped.")
            httpd.shutdown()
            sys.exit(0)

if __name__ == "__main__":
    main()