#!/usr/bin/env python3
"""
Emisor UDP: corre YOLOv8 sobre la webcam y envía, por cada frame, la mejor
detección como JSON a un puerto UDP (lo recibe el nodo ROS udp_bridge_node.py).

Mensaje enviado (un datagrama por frame):
{
  "seq":  int,    # contador de frames
  "t":    float,  # time.time() del emisor
  "det":  0|1,    # 1 si hay detección
  "cx":   float,  # centro x normalizado [-1, 1]  (+ = derecha de la imagen)
  "cy":   float,  # centro y normalizado [-1, 1]  (+ = abajo)
  "w":    float,  # ancho caja / ancho imagen  [0, 1]
  "h":    float,  # alto caja / alto imagen    [0, 1]
  "area": float,  # w * h                       [0, 1]
  "conf": float,  # confianza [0, 1]
  "cls":  int,    # id de clase (-1 si no hay detección)
  "follow": {...}, # mejor detección de la clase a seguir (taza)
  "avoid":  {...}  # mejor detección de la clase a evitar (naranja)
}
Los campos det..cls son la detección de mayor confianza (como siempre). "follow" y
"avoid" traen {"det","cx","cy","w","h","conf"} de la mejor caja de cada clase
(det=0 si esa clase no está en el frame), para que el controlador pueda decidir
cuando aparecen las dos a la vez.

Se envía también cuando NO hay detección (det=0), así el receptor distingue
"no veo el objeto" de "se cayó el emisor".

Uso:
  python3 yolo_udp_sender.py --model best.pt --source 0 --show
  python3 yolo_udp_sender.py --model best.pt --target pelota --port 5005
"""
import argparse
import json
import socket
import time

import cv2
from ultralytics import YOLO


def parse_args():
    p = argparse.ArgumentParser(description="YOLOv8 -> UDP")
    p.add_argument("--model", required=True, help="ruta a best.pt (u .onnx/.engine)")
    p.add_argument("--source", default="0", help="índice de cámara (0, 1, ...) o ruta a video")
    p.add_argument("--host", default="127.0.0.1", help="IP del receptor")
    p.add_argument("--port", type=int, default=5005, help="puerto UDP del receptor")
    p.add_argument("--conf", type=float, default=0.5, help="umbral de confianza")
    p.add_argument("--imgsz", type=int, default=640, help="tamaño de inferencia")
    p.add_argument("--device", default=None, help="'0' para GPU, 'cpu' para CPU (por defecto: auto)")
    p.add_argument("--target", default=None,
                   help="clase a seguir (nombre o id). Por defecto: cualquier clase")
    p.add_argument("--width", type=int, default=640, help="ancho pedido a la cámara")
    p.add_argument("--height", type=int, default=480, help="alto pedido a la cámara")
    p.add_argument("--follow-class", default="follow_obj", help="clase a seguir (nombre o id)")
    p.add_argument("--avoid-class", default="avoid_obj", help="clase a evitar (nombre o id)")
    p.add_argument("--show", action="store_true", help="mostrar ventana con las detecciones")
    return p.parse_args()


def resolve_target(model, target):
    """Convierte --target (nombre o id) en lista de ids para model.predict(classes=...)."""
    if target is None:
        return None
    names = model.names  # dict {id: nombre}
    if str(target).isdigit():
        cid = int(target)
        if cid not in names:
            raise SystemExit("Clase %d no existe. Clases: %s" % (cid, names))
        return [cid]
    for cid, name in names.items():
        if name.lower() == str(target).lower():
            return [cid]
    raise SystemExit("Clase '%s' no existe. Clases: %s" % (target, names))


def class_id(model, name_or_id):
    """Id de una clase por nombre o número; None si el modelo no la tiene."""
    names = model.names
    if str(name_or_id).isdigit():
        return int(name_or_id) if int(name_or_id) in names else None
    for cid, name in names.items():
        if name.lower() == str(name_or_id).lower():
            return cid
    return None


def open_source(source, width, height):
    src = int(source) if str(source).isdigit() else source
    cap = cv2.VideoCapture(src)
    if isinstance(src, int):
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # menos frames viejos en cola
    if not cap.isOpened():
        raise SystemExit("No se pudo abrir la fuente de video: %s" % source)
    return cap


def best_detection(result, frame_w, frame_h):
    """Devuelve el dict de la detección con mayor confianza, o uno con det=0."""
    boxes = result.boxes
    if boxes is None or len(boxes) == 0:
        return {"det": 0, "cx": 0.0, "cy": 0.0, "w": 0.0, "h": 0.0,
                "area": 0.0, "conf": 0.0, "cls": -1}

    i = int(boxes.conf.argmax())
    x1, y1, x2, y2 = boxes.xyxy[i].tolist()
    half_w, half_h = frame_w / 2.0, frame_h / 2.0
    bw = (x2 - x1) / frame_w
    bh = (y2 - y1) / frame_h
    return {
        "det": 1,
        "cx": ((x1 + x2) / 2.0 - half_w) / half_w,
        "cy": ((y1 + y2) / 2.0 - half_h) / half_h,
        "w": bw,
        "h": bh,
        "area": bw * bh,
        "conf": float(boxes.conf[i]),
        "cls": int(boxes.cls[i]),
    }


def class_detection(result, frame_w, frame_h, cid):
    """Mejor caja de la clase cid: {det, cx, cy, w, h, conf} (det=0 si no está)."""
    empty = {"det": 0, "cx": 0.0, "cy": 0.0, "w": 0.0, "h": 0.0, "conf": 0.0}
    boxes = result.boxes
    if cid is None or boxes is None or len(boxes) == 0:
        return empty
    best_i, best_conf = None, -1.0
    for i in range(len(boxes)):
        if int(boxes.cls[i]) == cid and float(boxes.conf[i]) > best_conf:
            best_i, best_conf = i, float(boxes.conf[i])
    if best_i is None:
        return empty
    x1, y1, x2, y2 = boxes.xyxy[best_i].tolist()
    half_w, half_h = frame_w / 2.0, frame_h / 2.0
    return {
        "det": 1,
        "cx": ((x1 + x2) / 2.0 - half_w) / half_w,
        "cy": ((y1 + y2) / 2.0 - half_h) / half_h,
        "w": (x2 - x1) / frame_w,
        "h": (y2 - y1) / frame_h,
        "conf": best_conf,
    }


def main():
    args = parse_args()
    model = YOLO(args.model)
    classes = resolve_target(model, args.target)
    follow_id = class_id(model, args.follow_class)
    avoid_id = class_id(model, args.avoid_class)
    if follow_id is None or avoid_id is None:
        print("AVISO: el modelo no tiene la clase %s o %s (clases: %s); ese campo irá siempre vacío."
              % (args.follow_class, args.avoid_class, model.names))
    cap = open_source(args.source, args.width, args.height)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    dest = (args.host, args.port)
    print("Enviando a %s:%d | clases: %s" % (args.host, args.port,
                                            classes if classes else "todas"))

    seq = 0
    fps, t_prev = 0.0, time.time()
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                print("Fin del video o error de cámara.")
                break

            frame_h, frame_w = frame.shape[:2]
            result = model.predict(frame, conf=args.conf, imgsz=args.imgsz,
                                   classes=classes, device=args.device,
                                   verbose=False)[0]

            msg = best_detection(result, frame_w, frame_h)
            msg["follow"] = class_detection(result, frame_w, frame_h, follow_id)
            msg["avoid"] = class_detection(result, frame_w, frame_h, avoid_id)
            msg["seq"] = seq
            msg["t"] = time.time()
            sock.sendto(json.dumps(msg).encode("utf-8"), dest)
            seq += 1

            # FPS suavizado
            now = time.time()
            dt = now - t_prev
            t_prev = now
            if dt > 0:
                fps = 0.9 * fps + 0.1 * (1.0 / dt) if fps else 1.0 / dt

            if args.show:
                vis = result.plot()
                cv2.putText(vis, "FPS %.1f  taza=%d naranja=%d" %
                            (fps, msg["follow"]["det"], msg["avoid"]["det"]),
                            (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                cv2.imshow("YOLO -> UDP", vis)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
            elif seq % 30 == 0:
                print("FPS %.1f | taza det=%d h=%.3f | naranja det=%d h=%.3f" %
                      (fps, msg["follow"]["det"], msg["follow"]["h"],
                       msg["avoid"]["det"], msg["avoid"]["h"]))
    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
        sock.close()
        if args.show:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
