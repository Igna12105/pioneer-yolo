#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Controlador follow / avoid para el Pioneer P3-DX (ROS Melodic, Python 2.7).

Entrada:  /yolo/detection  (std_msgs/Float32MultiArray)
          data = [det, cx, cy, w, h, area, conf, cls,      mejor detección
                  f_det, f_cx, f_h, f_conf,                taza    (follow)
                  a_det, a_cx, a_h, a_conf]                naranja (avoid)
          Con un emisor antiguo llegan solo los 8 primeros; en ese caso se usa
          la mejor detección según su clase.
Salida:   cmd_vel          (geometry_msgs/Twist)  -> remapeado a /RosAria/cmd_vel

Distancia (modelo pinhole, calibrado con calibrate_distance.py):
          d = f_px[clase] * alto_m[clase] / (h * image_height)

Prioridades (de mayor a menor):
  AVOID             naranja a menos de d_safe: retrocede manteniéndola centrada
                    (ignora la taza).
  FOLLOW_BLOQUEADO  taza visible y naranja a menos de d_block: gira hacia la
                    taza pero no avanza (sí puede retroceder).
  FOLLOW            taza visible: la centra y se acerca hasta d_target.
  AVOID_OK          solo naranja, lejos: quieto, girando para mantenerla centrada.
  LOST              nada (o confianza baja): frena.
  STALE             /yolo/detection no llega hace > msg_timeout: frena.

Seguridad: velocidades saturadas, rampas de aceleración y Twist cero al apagar.

Publica además (debug):
  ~state             (std_msgs/String)   estado actual
  ~distance          (std_msgs/Float32)  distancia al objeto que manda (-1 si ninguno)
  ~distance_follow   (std_msgs/Float32)  distancia filtrada a la taza (-1 si no está)
  ~distance_avoid    (std_msgs/Float32)  distancia filtrada a la naranja (-1 si no está)
"""
from __future__ import division

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Float32MultiArray, String

DET, CX, CY, W, H, AREA, CONF, CLS = range(8)
FOLLOW_OFF, AVOID_OFF = 8, 12   # inicio de [det, cx, h, conf] de cada clase


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def slew(current, target, max_up, max_down, dt):
    """Limita la variación de velocidad. max_up al acelerar (|v| crece), max_down al frenar."""
    delta = target - current
    growing = abs(target) > abs(current) and (target * current >= 0)
    rate = max_up if growing else max_down
    step = rate * dt
    return current + clamp(delta, -step, step)


class Params(object):
    """Lee todos los parámetros privados (~) con valores por defecto (modo lento)."""

    def __init__(self, get=None):
        get = get or rospy.get_param
        self.enabled = bool(get("~enabled", True))
        self.rate = float(get("~rate", 15.0))
        self.msg_timeout = float(get("~msg_timeout", 0.5))
        self.min_conf = float(get("~min_conf", 0.5))

        # Calibración (de calibracion.yaml)
        self.image_height = float(get("~image_height", 480))
        self.follow_class = int(get("~follow_class_id", 1))
        self.avoid_class = int(get("~avoid_class_id", 0))
        self.f_px = {self.follow_class: float(get("~f_px_follow", 302.14)),
                     self.avoid_class: float(get("~f_px_avoid", 321.91))}
        self.height_m = {self.follow_class: float(get("~height_follow_m", 0.10)),
                         self.avoid_class: float(get("~height_avoid_m", 0.08))}

        # Objetivos de distancia [m]
        self.d_target = float(get("~d_target", 0.30))
        self.d_deadband = float(get("~d_deadband", 0.03))
        self.d_safe = float(get("~d_safe", 0.60))
        self.d_safe_hyst = float(get("~d_safe_hysteresis", 0.05))
        self.d_block = float(get("~d_block", 0.80))
        self.d_block_hyst = float(get("~d_block_hysteresis", 0.05))

        # Ganancias y límites
        self.k_lin = float(get("~k_lin", 0.6))
        self.k_ang = float(get("~k_ang", 0.5))
        self.cx_deadband = float(get("~cx_deadband", 0.05))
        self.cx_slow = float(get("~cx_slow", 0.5))  # |cx| a partir del cual no avanza
        self.v_max = float(get("~v_max", 0.10))
        self.v_back_max = float(get("~v_back_max", 0.08))
        self.w_max = float(get("~w_max", 0.30))
        self.lin_accel = float(get("~lin_accel", 0.2))
        self.lin_decel = float(get("~lin_decel", 0.8))
        self.ang_accel = float(get("~ang_accel", 1.0))
        self.ang_decel = float(get("~ang_decel", 4.0))

        # Filtro exponencial (1 = sin filtro)
        self.alpha_d = float(get("~alpha_distance", 0.4))
        self.alpha_cx = float(get("~alpha_cx", 0.5))


class Track(object):
    """Distancia y cx filtrados de UNA clase. Se reinicia cuando la clase desaparece."""

    def __init__(self, alpha_d, alpha_cx):
        self.alpha_d, self.alpha_cx = alpha_d, alpha_cx
        self.d = None
        self.cx = None

    def reset(self):
        self.d = None
        self.cx = None

    def update(self, d, cx):
        if self.d is None:
            self.d, self.cx = d, cx
        else:
            self.d += self.alpha_d * (d - self.d)
            self.cx += self.alpha_cx * (cx - self.cx)


class FollowAvoidLogic(object):
    """Lógica pura (sin ROS) para poder probarla aparte."""

    def __init__(self, p):
        self.p = p
        self.follow = Track(p.alpha_d, p.alpha_cx)
        self.avoid = Track(p.alpha_d, p.alpha_cx)
        self.avoiding = False   # histéresis de d_safe
        self.blocked = False    # histéresis de d_block

    def reset(self):
        self.follow.reset()
        self.avoid.reset()
        self.avoiding = False
        self.blocked = False

    def _distance(self, h_frac, cls):
        h_px = h_frac * self.p.image_height
        if h_px <= 1.0:
            return None
        return self.p.f_px[cls] * self.p.height_m[cls] / h_px

    def _split(self, data):
        """data -> {cls: (cx, h)} con las clases visibles y con confianza suficiente."""
        objs = {}
        if data is None:
            return objs
        if len(data) >= 16:
            for cls, off in ((self.p.follow_class, FOLLOW_OFF), (self.p.avoid_class, AVOID_OFF)):
                det, cx, h, conf = data[off:off + 4]
                if det >= 0.5 and conf >= self.p.min_conf:
                    objs[cls] = (cx, h)
        elif len(data) >= 8 and data[DET] >= 0.5 and data[CONF] >= self.p.min_conf:
            cls = int(round(data[CLS]))
            if cls in (self.p.follow_class, self.p.avoid_class):
                objs[cls] = (data[CX], data[H])
        return objs

    def _update_track(self, track, objs, cls):
        """Actualiza el filtro de una clase; devuelve (d, cx) filtrados o (None, None)."""
        d = None
        if cls in objs:
            cx, h = objs[cls]
            d = self._distance(h, cls)
            if d is not None:
                track.update(d, cx)
        if d is None:
            track.reset()
            return None, None
        return track.d, track.cx

    def _turn(self, cx):
        """cx > 0 (derecha) => w < 0 (girar a la derecha)."""
        w = 0.0 if abs(cx) < self.p.cx_deadband else -self.p.k_ang * cx
        return clamp(w, -self.p.w_max, self.p.w_max)

    def step(self, data):
        """Devuelve (v, w, estado, d_principal, d_taza, d_naranja)."""
        p = self.p
        objs = self._split(data)
        d_f, cx_f = self._update_track(self.follow, objs, p.follow_class)
        d_a, cx_a = self._update_track(self.avoid, objs, p.avoid_class)

        # Histéresis de las zonas de la naranja (sin naranja visible, se apagan)
        if d_a is None:
            self.avoiding = False
            self.blocked = False
        else:
            self.avoiding = d_a < p.d_safe or (self.avoiding and d_a < p.d_safe + p.d_safe_hyst)
            self.blocked = d_a < p.d_block or (self.blocked and d_a < p.d_block + p.d_block_hyst)

        # 1. Seguridad: naranja cerca => retroceder (ignora la taza)
        if self.avoiding:
            v = -p.k_lin * (p.d_safe + p.d_safe_hyst - d_a)
            v = clamp(v, -p.v_back_max, 0.0)
            return v, self._turn(cx_a), "AVOID", d_a, d_f, d_a

        # 2. Taza visible => seguirla (sin avanzar si la naranja está en la zona de bloqueo)
        if d_f is not None:
            err = d_f - p.d_target
            v = 0.0 if abs(err) < p.d_deadband else p.k_lin * err
            if v > 0:  # avanzar solo si está razonablemente centrada
                v *= max(0.0, 1.0 - abs(cx_f) / p.cx_slow)
            v = clamp(v, -p.v_back_max, p.v_max)
            state = "FOLLOW"
            if self.blocked:
                v = min(v, 0.0)
                state = "FOLLOW_BLOQUEADO"
            return v, self._turn(cx_f), state, d_f, d_f, d_a

        # 3. Solo naranja, lejos => quieto mirándola
        if d_a is not None:
            return 0.0, self._turn(cx_a), "AVOID_OK", d_a, d_f, d_a

        # 4. Nada
        return 0.0, 0.0, "LOST", None, None, None


def as_float(d):
    return Float32(data=d if d is not None else -1.0)


class FollowAvoidNode(object):
    def __init__(self):
        self.p = Params()
        self.logic = FollowAvoidLogic(self.p)
        self.last_data = None
        self.last_rx = None
        self.v = 0.0
        self.w = 0.0

        self.pub_cmd = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.pub_state = rospy.Publisher("~state", String, queue_size=1)
        self.pub_dist = rospy.Publisher("~distance", Float32, queue_size=1)
        self.pub_dist_f = rospy.Publisher("~distance_follow", Float32, queue_size=1)
        self.pub_dist_a = rospy.Publisher("~distance_avoid", Float32, queue_size=1)
        rospy.Subscriber("detection", Float32MultiArray, self.on_detection, queue_size=1)
        rospy.on_shutdown(self.stop)

        rospy.loginfo("follow_avoid: d_target=%.2fm d_safe=%.2fm d_block=%.2fm v_max=%.2f "
                      "w_max=%.2f enabled=%s", self.p.d_target, self.p.d_safe, self.p.d_block,
                      self.p.v_max, self.p.w_max, self.p.enabled)

    def on_detection(self, msg):
        if len(msg.data) >= 8:
            self.last_data = list(msg.data)
            self.last_rx = rospy.Time.now()

    def stop(self):
        self.pub_cmd.publish(Twist())

    def spin(self):
        rate = rospy.Rate(self.p.rate)
        dt = 1.0 / self.p.rate
        last_state = None
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            stale = (self.last_rx is None or
                     (now - self.last_rx).to_sec() > self.p.msg_timeout)

            if not self.p.enabled:
                v_t, w_t, state, d, d_f, d_a = 0.0, 0.0, "DISABLED", None, None, None
            elif stale:
                self.logic.reset()
                v_t, w_t, state, d, d_f, d_a = 0.0, 0.0, "STALE", None, None, None
            else:
                v_t, w_t, state, d, d_f, d_a = self.logic.step(self.last_data)

            # Rampas: acelera suave, frena rápido
            self.v = slew(self.v, v_t, self.p.lin_accel, self.p.lin_decel, dt)
            self.w = slew(self.w, w_t, self.p.ang_accel, self.p.ang_decel, dt)
            if abs(self.v) < 1e-3:
                self.v = 0.0
            if abs(self.w) < 1e-3:
                self.w = 0.0

            cmd = Twist()
            cmd.linear.x = self.v
            cmd.angular.z = self.w
            self.pub_cmd.publish(cmd)
            self.pub_state.publish(String(data=state))
            self.pub_dist.publish(as_float(d))
            self.pub_dist_f.publish(as_float(d_f))
            self.pub_dist_a.publish(as_float(d_a))

            if state != last_state:
                rospy.loginfo("estado: %s", state)
                last_state = state
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("follow_avoid_controller")
    try:
        FollowAvoidNode().spin()
    except rospy.ROSInterruptException:
        pass
