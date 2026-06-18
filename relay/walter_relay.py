#!/usr/bin/env python3
"""
Walter Field Node UDP/HTTP Relay Server
Runs on a cloud VPS (Linode). Bridges two transport paths from the Walter device:

  - UDP port 20007  -- cellular path (Walter -> Soracom LTE -> this server)
  - HTTP port 20008 -- WiFi failover path (Walter -> local WiFi -> internet -> this server)

Both paths forward the JSON telemetry to a Debian receiver over Tailscale.
Payloads are tagged with "path": "cell" or "path": "wifi" before forwarding
so the receiver can distinguish transport methods in the database.

Architecture:
  Walter (ESP32-S3)
    |-- LTE/Soracom --> UDP:20007 --> this relay --> Tailscale --> Debian receiver
    |-- WiFi UDP    --> HTTP:20008 -> this relay --> Tailscale --> Debian receiver
"""
import socket
import json
import logging
import threading
import requests
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

LISTEN_HOST  = "0.0.0.0"
UDP_PORT     = 20007   # cellular path
HTTP_PORT    = 20008   # wifi failover path
FORWARD_URL  = "http://your-tailscale-receiver:8081/walter"  # configure for your network
LOG_FILE     = "/home/kf7kgn/walter_relay.log"

logging.basicConfig(
    filename=LOG_FILE,
    level=logging.INFO,
    format="[%(asctime)s] %(message)s",
    datefmt="%Y-%m-%dT%H:%M:%S%z",
)


def forward_to_receiver(payload_dict, source_label):
    """POST the path-tagged JSON payload to the receiver over Tailscale."""
    try:
        r = requests.post(FORWARD_URL, json=payload_dict, timeout=5)
        logging.info(f"  [{source_label}] forwarded -> HTTP {r.status_code}")
        return r.status_code
    except Exception as e:
        logging.error(f"  [{source_label}] forward FAILED: {e}")
        return None


def udp_listener():
    """
    Cellular path listener.
    Walter sends raw UDP JSON packets from the field via Soracom LTE.
    Tags payload with path=cell and src_ip before forwarding.
    """
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((LISTEN_HOST, UDP_PORT))
    logging.info(f"UDP listener up on :{UDP_PORT} (cellular path)")
    print(f"UDP listener bound to :{UDP_PORT}", flush=True)

    while True:
        try:
            data, addr = sock.recvfrom(2048)
            text = data.decode("utf-8", errors="replace").strip()
            logging.info(f"[CELL] recv {len(data)}B from {addr[0]}: {text}")

            try:
                payload = json.loads(text)
            except json.JSONDecodeError:
                logging.error(f"  [CELL] non-JSON packet, skipping")
                continue

            if isinstance(payload, dict):
                payload["path"] = "cell"
                payload["src_ip"] = addr[0]

            forward_to_receiver(payload, "CELL")

        except Exception as e:
            logging.error(f"  [CELL] error: {e}")


class WifiHandler(BaseHTTPRequestHandler):
    """
    WiFi failover path handler.
    Walter POSTs JSON to this HTTP endpoint when cellular is down or fails.
    The same JSON payload format is used for both transports.
    """

    def log_message(self, fmt, *args):
        # Suppress default HTTP access log; we use our own logger
        pass

    def do_GET(self):
        if self.path == "/health":
            self.send_response(200)
            self.send_header("Content-Type", "text/plain")
            self.end_headers()
            self.wfile.write(b"ok\n")
        else:
            self.send_response(404)
            self.end_headers()

    def do_POST(self):
        try:
            length = int(self.headers.get("Content-Length", 0))
            raw = self.rfile.read(length)
            text = raw.decode("utf-8", errors="replace").strip()
            src_ip = self.client_address[0]
            logging.info(f"[WIFI] recv {len(raw)}B from {src_ip}: {text}")

            try:
                payload = json.loads(text)
            except json.JSONDecodeError:
                self.send_response(400)
                self.end_headers()
                self.wfile.write(b'{"error":"not json"}')
                logging.error(f"  [WIFI] non-JSON, rejecting")
                return

            if isinstance(payload, dict):
                payload["path"] = "wifi"
                payload["src_ip"] = src_ip

            status = forward_to_receiver(payload, "WIFI")

            if status and 200 <= status < 300:
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.end_headers()
                self.wfile.write(b'{"ok":true}')
            else:
                self.send_response(502)
                self.end_headers()
                self.wfile.write(b'{"error":"forward failed"}')

        except Exception as e:
            logging.error(f"  [WIFI] handler error: {e}")
            try:
                self.send_response(500)
                self.end_headers()
            except Exception:
                pass


def http_listener():
    server = ThreadingHTTPServer((LISTEN_HOST, HTTP_PORT), WifiHandler)
    logging.info(f"HTTP listener up on :{HTTP_PORT} (wifi path)")
    print(f"HTTP listener bound to :{HTTP_PORT}", flush=True)
    server.serve_forever()


if __name__ == "__main__":
    # HTTP listener runs in a daemon thread; UDP runs in main thread
    threading.Thread(target=http_listener, daemon=True).start()
    udp_listener()
