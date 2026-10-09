#!/usr/bin/env python3
"""
Rover Android App Launcher
==========================
1. Detects local Wi-Fi IP address.
2. Checks and launches rosbridge_server (port 9090) if not running.
3. Hosts the web application on port 8000.
4. Generates a live QR Code in the terminal for instant phone connection.
"""

import http.server
import os
import socket
import subprocess
import sys
import threading
import time

try:
    import qrcode
except ImportError:
    qrcode = None


def get_local_ip():
    """Find local Wi-Fi IP on subnet (typically 192.168.1.x)."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        # Connect to common gateway / DNS without sending data
        s.connect(('192.168.1.1', 80))
        ip = s.getsockname()[0]
    except Exception:
        try:
            s.connect(('8.8.8.8', 80))
            ip = s.getsockname()[0]
        except Exception:
            ip = '127.0.0.1'
    finally:
        s.close()
    return ip


def is_port_in_use(port):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        return s.connect_ex(('127.0.0.1', port)) == 0


def launch_rosbridge():
    """Start ROS 2 rosbridge_server if port 9090 is not open."""
    if is_port_in_use(9090):
        print("[INFO] ROS Bridge WebSocket is already active on port 9090.")
        return None

    print("[INFO] Starting ROS 2 rosbridge_websocket on port 9090...")
    cmd = [
        "ros2", "launch", "rosbridge_server", "rosbridge_websocket_launch.xml",
        "port:=9090"
    ]
    proc = subprocess.Popen(
        cmd,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        preexec_fn=os.setsid
    )
    # Wait up to 4 seconds for port to open
    for _ in range(20):
        time.sleep(0.2)
        if is_port_in_use(9090):
            print("[SUCCESS] ROS Bridge is ready on port 9090.")
            return proc
    print("[WARNING] ROS Bridge started, waiting for WebSocket to listen...")
    return proc


def print_banner(ip, port):
    app_url = f"http://{ip}:{port}"
    print("\n" + "=" * 65)
    print("      🚀 ROVER MISSION CONTROL - ANDROID APP SERVER")
    print("=" * 65)
    print(f"\n📱 Connect your Android Phone (on the same Wi-Fi) to:")
    print(f"\n      ➡️   \033[1;36m{app_url}\033[0m\n")

    if qrcode:
        print("📲 Scan this QR Code with your phone camera to open:")
        qr = qrcode.QRCode(border=1)
        qr.add_data(app_url)
        qr.print_ascii(invert=True)
    else:
        print("Tip: Install 'qrcode' (pip install qrcode) to see a QR code.")

    print("-" * 65)
    print("Features ready on your Android phone:")
    print("  • 🗺️  Live Map & Costmap Visualization (/map)")
    print("  • 🎯 2D Pose Estimation (Touch & drag to set AMCL pose)")
    print("  • 📍 2D Nav Goal (Touch & drag to send destination)")
    print("  • 🚩 Waypoint Missions (Add, save, and run waypoint routes)")
    print("  • 🕹️  Virtual Touch Joystick (/cmd_vel_smoothed)")
    print("  • 🛑 Emergency Stop Button")
    print("  • 📲 PWA Support: Tap 'Add to Home Screen' in Chrome for Fullscreen")
    print("-" * 65)
    print("Press Ctrl+C to stop the server.\n")


def main():
    app_dir = os.path.dirname(os.path.abspath(__file__))
    os.chdir(app_dir)

    local_ip = get_local_ip()
    http_port = 8000

    rosbridge_proc = launch_rosbridge()

    class QuietHandler(http.server.SimpleHTTPRequestHandler):
        def log_message(self, format, *args):
            pass  # Suppress HTTP request logging spam

    server = http.server.ThreadingHTTPServer(('0.0.0.0', http_port), QuietHandler)
    server_thread = threading.Thread(target=server.serve_forever, daemon=True)
    server_thread.start()

    print_banner(local_ip, http_port)

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n[INFO] Shutting down Rover Android Server...")
        server.shutdown()
        if rosbridge_proc:
            try:
                import signal
                os.killpg(os.getpgid(rosbridge_proc.pid), signal.SIGTERM)
            except Exception:
                pass
        print("[INFO] Server stopped safely.")


if __name__ == '__main__':
    main()
