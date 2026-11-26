#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Nodo ROS (Melodic, Python 2.7): recibe por UDP las detecciones que envía
yolo_udp_sender.py y las publica en ROS.

Publica:
  detection (std_msgs/Float32MultiArray), remapeado a /yolo/detection
      data = [det, cx, cy, w, h, area, conf, cls,      <- mejor detección (como siempre)
              f_det, f_cx, f_h, f_conf,                <- taza    (follow)
              a_det, a_cx, a_h, a_conf]                <- naranja (avoid)
      idx:     0..7 | 8..11 | 12..15
      Si el emisor es antiguo (sin "follow"/"avoid") publica solo los 8 primeros.
  detected (std_msgs/Bool), remapeado a /yolo/detected

Seguridad: si no llega ningún paquete en `~timeout` segundos (emisor caído,
red, etc.) publica det=0 a `~stale_rate` Hz, así el controlador se detiene.

Parámetros:
  ~bind_ip    (str,   "0.0.0.0")
  ~port       (int,   5005)
  ~timeout    (float, 0.3)   s sin paquetes => se considera perdido
  ~stale_rate (float, 10.0)  Hz de publicación de det=0 mientras está perdido
"""
import json
import socket

import rospy
from std_msgs.msg import Bool, Float32MultiArray, MultiArrayDimension

FIELDS = ["det", "cx", "cy", "w", "h", "area", "conf", "cls"]
OBJ_FIELDS = ["det", "cx", "h", "conf"]
EXTRA_LABELS = ["f_det", "f_cx", "f_h", "f_conf", "a_det", "a_cx", "a_h", "a_conf"]


class UdpBridge(object):
    def __init__(self):
        self.bind_ip = rospy.get_param("~bind_ip", "0.0.0.0")
        self.port = int(rospy.get_param("~port", 5005))
        self.timeout = float(rospy.get_param("~timeout", 0.3))
        self.stale_period = 1.0 / float(rospy.get_param("~stale_rate", 10.0))

        self.pub_det = rospy.Publisher("detection", Float32MultiArray, queue_size=1)
        self.pub_flag = rospy.Publisher("detected", Bool, queue_size=1)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((self.bind_ip, self.port))
        self.sock.settimeout(0.05)  # para poder revisar timeout y rospy.is_shutdown()

        self.last_rx = None          # rospy.Time del último paquete válido
        self.last_stale_pub = rospy.Time(0)
        self.lost = True

        rospy.loginfo("udp_bridge escuchando en %s:%d (timeout %.2fs)",
                      self.bind_ip, self.port, self.timeout)

    def _drain(self, first):
        """Lee todo lo que haya en cola (evita procesar detecciones atrasadas)."""
        packets = [first]
        self.sock.setblocking(False)
        try:
            while True:
                packets.append(self.sock.recv(4096))
        except socket.error:
            pass
        finally:
            self.sock.settimeout(0.05)
        return packets

    @staticmethod
    def _parse(data):
        try:
            d = json.loads(data.decode("utf-8"))
            values = [float(d[k]) for k in FIELDS]
            if isinstance(d.get("follow"), dict) and isinstance(d.get("avoid"), dict):
                for key in ("follow", "avoid"):
                    values += [float(d[key][k]) for k in OBJ_FIELDS]
            return values
        except (ValueError, KeyError, TypeError, AttributeError):
            return None

    def _publish(self, values):
        msg = Float32MultiArray()
        dim = MultiArrayDimension()
        labels = FIELDS + (EXTRA_LABELS if len(values) > len(FIELDS) else [])
        dim.label = ",".join(labels)
        dim.size = len(labels)
        dim.stride = len(labels)
        msg.layout.dim.append(dim)
        msg.data = [float(v) for v in values]
        self.pub_det.publish(msg)
        self.pub_flag.publish(Bool(data=bool(values[0] > 0.5)))

    def _handle_packets(self, packets):
        # Usa el paquete válido más reciente
        values = None
        for data in reversed(packets):
            values = self._parse(data)
            if values is not None:
                break
        if values is None:
            rospy.logwarn_throttle(2.0, "Paquete(s) UDP inválido(s), ignorado(s)")
            return
        self.last_rx = rospy.Time.now()
        if self.lost:
            rospy.loginfo("Stream UDP recibido")
            self.lost = False
        self._publish(values)

    def _check_timeout(self):
        now = rospy.Time.now()
        expired = (self.last_rx is None or
                   (now - self.last_rx).to_sec() > self.timeout)
        if not expired:
            return
        if not self.lost:
            rospy.logwarn("Sin paquetes UDP hace más de %.2fs -> det=0", self.timeout)
            self.lost = True
        if (now - self.last_stale_pub).to_sec() >= self.stale_period:
            self._publish([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0] + [0.0] * len(EXTRA_LABELS))
            self.last_stale_pub = now

    def spin(self):
        while not rospy.is_shutdown():
            try:
                data = self.sock.recv(4096)
                self._handle_packets(self._drain(data))
            except socket.timeout:
                pass
            except socket.error as e:
                rospy.logwarn_throttle(2.0, "Error de socket: %s" % e)
            self._check_timeout()
        self.sock.close()


if __name__ == "__main__":
    rospy.init_node("udp_bridge")
    UdpBridge().spin()
