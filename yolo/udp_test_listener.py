#!/usr/bin/env python3
"""
Receptor de prueba SIN ROS: imprime lo que llega por UDP.
Sirve para validar el emisor antes de meterse con Docker/ROS.

  python3 udp_test_listener.py --port 5005
"""
import argparse
import json
import socket
import time

p = argparse.ArgumentParser()
p.add_argument("--port", type=int, default=5005)
args = p.parse_args()

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", args.port))
print("Escuchando en UDP :%d (Ctrl+C para salir)" % args.port)

try:
    while True:
        data, _ = sock.recvfrom(4096)
        msg = json.loads(data.decode("utf-8"))
        lat_ms = (time.time() - msg.get("t", time.time())) * 1000.0
        print("seq=%6d det=%d cx=%+.2f cy=%+.2f area=%.3f conf=%.2f lat=%.1fms" %
              (msg["seq"], msg["det"], msg["cx"], msg["cy"],
               msg["area"], msg["conf"], lat_ms))
except KeyboardInterrupt:
    pass
finally:
    sock.close()
