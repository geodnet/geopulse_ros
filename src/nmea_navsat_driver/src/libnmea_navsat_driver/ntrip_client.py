#!/usr/bin/env python3
"""
NTRIP client for injecting RTCM corrections into a GNSS device via serial port.
Connects to an NTRIP caster, streams RTCM data, and writes it to the serial port.
"""

import socket
import base64
import threading
import time
import logging

logger = logging.getLogger(__name__)


class NtripClient:
    """
    Connects to an NTRIP caster and streams RTCM corrections to a serial port.
    Runs in a background thread. Automatically reconnects on failure.
    """

    NTRIP_VERSION = "2.0"
    BUFFER_SIZE = 4096
    RECONNECT_DELAY = 5.0  # seconds between reconnect attempts

    def __init__(self, host, port, mountpoint, username, password, serial_port, logger=None, rtcm_callback=None, position_callback=None):
        self.host = host
        self.port = port
        self.mountpoint = mountpoint
        self.username = username
        self.password = password
        self.serial_port = serial_port  # serial.Serial instance shared with driver
        self.logger = logger or logging.getLogger(__name__)

        self.thread = None
        self.stop_event = threading.Event()
        self.connected = False
        self.bytes_received = 0
        self.rtcm_callback = rtcm_callback
        self.position_callback = position_callback

    def start(self):
        """Start the NTRIP client in a background thread."""
        self.stop_event.clear()
        self.thread = threading.Thread(target=self.run, daemon=True, name="ntrip_client")
        self.thread.start()
        self.logger.info(f"NTRIP client started: {self.host}:{self.port}/{self.mountpoint}")

    def stop(self):
        """Stop the NTRIP client."""
        self.stop_event.set()
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=5.0)
        self.logger.info("NTRIP client stopped.")

    @property
    def is_connected(self):
        return self.connected

    def build_request(self):
        credentials = base64.b64encode(
            f"{self.username}:{self.password}".encode()
        ).decode()

        request = (
            f"GET /{self.mountpoint} HTTP/1.0\r\n"
            f"User-Agent: NTRIP ROS2Driver/1.0\r\n"
            f"Authorization: Basic {credentials}\r\n"
            f"\r\n"
        )
        return request.encode()

    def connect(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(10.0)
        sock.connect((self.host, self.port))

        sock.sendall(self.build_request())

        if self.position_callback:
            lat, lon, alt = self.position_callback()
        else:
            lat, lon, alt = 0.0, 0.0, 0.0

        lat_deg = int(abs(lat))
        lat_min = (abs(lat) - lat_deg) * 60
        lat_str = f"{lat_deg:02d}{lat_min:07.4f}"
        lat_hem = "N" if lat >= 0 else "S"

        lon_deg = int(abs(lon))
        lon_min = (abs(lon) - lon_deg) * 60
        lon_str = f"{lon_deg:03d}{lon_min:07.4f}"
        lon_hem = "E" if lon >= 0 else "W"

        gga = f"$GPGGA,000000.00,{lat_str},{lat_hem},{lon_str},{lon_hem},1,12,1.0,{alt:.1f},M,0.0,M,,*00\r\n"
        sock.sendall(gga.encode())

        # Read response - NTRIP v1 may not send \r\n\r\n
        response = b""
        while True:
            chunk = sock.recv(1024)
            if not chunk:
                raise ConnectionError("Connection closed before response received")
            response += chunk
            if b"\r\n" in response:
                break

        first_line = response.split(b"\r\n")[0].decode(errors="ignore")

        if "200 OK" not in first_line and "ICY 200 OK" not in first_line:
            raise ConnectionError(f"NTRIP caster rejected connection: {first_line}")

        self.logger.info(f"Connected to NTRIP caster: {first_line}")

        leftover = response.split(b"\r\n", 1)[1] if b"\r\n" in response else b""
        return sock, leftover

    def run(self):
        """Main loop: connect, stream RTCM, reconnect on failure."""
        while not self.stop_event.is_set():
            sock = None
            try:
                self.connected = False
                sock, leftover = self.connect()
                sock.settimeout(5.0)
                self.connected = True

                # Write any leftover data from header read
                if leftover:
                    self.write_to_serial(leftover)

                # Stream RTCM data
                while not self.stop_event.is_set():
                    try:
                        data = sock.recv(self.BUFFER_SIZE)
                        if not data:
                            raise ConnectionError("NTRIP caster closed connection")
                        self.write_to_serial(data)
                    except socket.timeout:
                        continue  # Normal, just retry

            except Exception as e:
                self.connected = False
                self.logger.warning(
                    f"NTRIP connection error: {e}. "
                    f"Reconnecting in {self.RECONNECT_DELAY}s..."
                )
            finally:
                if sock:
                    try:
                        sock.close()
                    except Exception:
                        pass
                self.connected = False

            # Wait before reconnecting
            self.stop_event.wait(self.RECONNECT_DELAY)

    def write_to_serial(self, data):
        """Write RTCM bytes to the serial port and publish to ROS topic."""
        try:
            if self.serial_port and self.serial_port.is_open:
                self.serial_port.write(data)
                self.bytes_received += len(data)
                self.logger.debug(f"Injected {len(data)} RTCM bytes to device")
                # Publish to ROS topic if callback is set
                if self.rtcm_callback:
                    self.rtcm_callback(data)
        except Exception as e:
            self.logger.error(f"Failed to write RTCM to serial port: {e}")