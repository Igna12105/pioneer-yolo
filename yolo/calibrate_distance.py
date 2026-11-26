#!/usr/bin/env python3
"""
Calibración de distancia para el Proyecto Final EL5206 (Pioneer + YOLOv8).

Modelo de cámara estenopeica (pinhole):

        d = f_px * H_real / h_px

    d       distancia cámara -> objeto [m]
    f_px    distancia focal de la cámara en píxeles (lo que se calibra)
    H_real  alto real del objeto [m]   (taza 0.10, naranja 0.08)
    h_px    alto de la caja de YOLO en píxeles

Procedimiento:
  1. Pones cada objeto a distancias conocidas (huincha), uno a la vez.
  2. En cada punto el script junta N frames, toma la mediana de h_px y calcula
     f_px = h_px * d / H_real.
  3. Promedia f_px y guarda todo en calibracion.yaml.
  4. Queda en modo verificación: muestra en vivo la distancia estimada.

Mide siempre igual: desde el lente de la cámara hasta el frente del objeto.
Calibra en la resolución que vas a usar en el robot (por defecto 640x480) y
repite la calibración cuando la cámara quede montada en el robot.

Uso:
  python calibrate_distance.py                          # calibrar (valores por defecto)
  python calibrate_distance.py --distances 30,50,75,100,150
  python calibrate_distance.py --check                  # solo verificar con calibracion.yaml

Teclas (con la ventana seleccionada):
  ESPACIO  medir el punto actual        n  saltar el punto
  r        repetir el punto anterior    q  terminar
"""
import argparse
import datetime
import statistics
import time
from pathlib import Path

import cv2
import yaml
from ultralytics import YOLO

HERE = Path(__file__).resolve().parent
DEFAULT_MODEL = HERE / "model" / "best.pt"
DEFAULT_OUT = HERE / "calibracion.yaml"
DEFAULT_HEIGHTS = "follow_obj=0.10,avoid_obj=0.08"   # taza 10 cm, naranja 8 cm
BORDER_PX = 3   # caja a menos de esto del borde = objeto cortado -> se descarta


# ----------------------------------------------------------------------------
# Utilidades
# ----------------------------------------------------------------------------
def parse_args():
    p = argparse.ArgumentParser(description="Calibración de distancia (pinhole) con YOLOv8")
    p.add_argument("--model", default=str(DEFAULT_MODEL), help="ruta a best.pt")
    p.add_argument("--source", default="2", help="índice de cámara o ruta /dev/v4l/by-id/...")
    p.add_argument("--width", type=int, default=640)
    p.add_argument("--height", type=int, default=480)
    p.add_argument("--conf", type=float, default=0.5)
    p.add_argument("--imgsz", type=int, default=640)
    p.add_argument("--device", default=None, help="'0' GPU, 'cpu' CPU (auto por defecto)")
    p.add_argument("--heights", default=DEFAULT_HEIGHTS,
                   help="altos reales en metros: 'clase=alto,clase=alto'")
    p.add_argument("--distances", default="30,50,75,100,150",
                   help="distancias de calibración en cm, separadas por coma")
    p.add_argument("--frames", type=int, default=30, help="frames por punto")
    p.add_argument("--timeout", type=float, default=10.0, help="s máximos por punto")
    p.add_argument("--out", default=str(DEFAULT_OUT), help="archivo YAML de salida")
    p.add_argument("--check", action="store_true",
                   help="no calibrar: cargar --out y mostrar distancia en vivo")
    return p.parse_args()


def parse_heights(text, names):
    """'follow_obj=0.10,avoid_obj=0.08' -> {cls_id: (nombre, alto_m)} validado contra el modelo."""
    by_name = {n.lower(): cid for cid, n in names.items()}
    out = {}
    for item in text.split(","):
        item = item.strip()
        if not item:
            continue
        name, _, value = item.partition("=")
        name = name.strip()
        if name.lower() not in by_name:
            raise SystemExit("Clase '%s' no existe en el modelo. Clases: %s" % (name, names))
        h = float(value)
        if not 0.005 < h < 5.0:
            raise SystemExit("Alto de '%s' fuera de rango (%.3f m). ¿Lo pusiste en metros?" % (name, h))
        cid = by_name[name.lower()]
        out[cid] = (names[cid], h)
    if not out:
        raise SystemExit("--heights vacío")
    return out


def open_camera(source, width, height):
    if str(source).isdigit():
        cap = cv2.VideoCapture(int(source), cv2.CAP_V4L2)
    else:
        cap = cv2.VideoCapture(source, cv2.CAP_V4L2)
    if not cap.isOpened():
        raise SystemExit("No se pudo abrir la cámara %s (¿índice correcto? ¿grupo video? "
                         "¿otro programa la está usando?)" % source)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    ok, frame = cap.read()
    if not ok:
        raise SystemExit("La cámara abrió pero no entrega frames.")
    real_h, real_w = frame.shape[:2]
    if (real_w, real_h) != (width, height):
        print("AVISO: pediste %dx%d pero la cámara entrega %dx%d. Se calibra a %dx%d."
              % (width, height, real_w, real_h, real_w, real_h))
    return cap, real_w, real_h


def best_box_per_class(result):
    """{cls_id: (conf, x1, y1, x2, y2)} con la caja de mayor confianza de cada clase."""
    best = {}
    boxes = result.boxes
    if boxes is None or len(boxes) == 0:
        return best
    for xyxy, conf, cls in zip(boxes.xyxy.tolist(), boxes.conf.tolist(), boxes.cls.tolist()):
        cid = int(cls)
        if cid not in best or conf > best[cid][0]:
            best[cid] = (conf, *xyxy)
    return best


def is_truncated(box, img_w, img_h):
    _, x1, y1, x2, y2 = box
    return (x1 <= BORDER_PX or y1 <= BORDER_PX or
            x2 >= img_w - 1 - BORDER_PX or y2 >= img_h - 1 - BORDER_PX)


def distance_m(f_px, height_m, h_px):
    return f_px * height_m / h_px if h_px > 0 else float("inf")


def put(img, text, y, color=(255, 255, 255), scale=0.6):
    cv2.putText(img, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), 4)
    cv2.putText(img, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX, scale, color, 2)


def draw_boxes(img, boxes, heights, img_w, img_h, f_px=None):
    """Dibuja las cajas con h_px y, si hay f_px, la distancia estimada."""
    for cid, box in boxes.items():
        conf, x1, y1, x2, y2 = box
        h_px = y2 - y1
        cut = is_truncated(box, img_w, img_h)
        color = (0, 0, 255) if cut else (0, 255, 0)
        cv2.rectangle(img, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)
        name = heights[cid][0] if cid in heights else str(cid)
        label = "%s %.2f h=%.0fpx" % (name, conf, h_px)
        if f_px and cid in heights:
            label += "  d=%.2fm" % distance_m(f_px, heights[cid][1], h_px)
        if cut:
            label += " CORTADO"
        cv2.putText(img, label, (int(x1), max(15, int(y1) - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)


# ----------------------------------------------------------------------------
# Cálculo y guardado
# ----------------------------------------------------------------------------
def summarize(samples, heights):
    """Calcula f_px global y por clase, e imprime el error de distancia de cada punto."""
    if not samples:
        raise SystemExit("No se midió ningún punto; no hay nada que guardar.")
    f_all = [s["f_px"] for s in samples]
    f_px = statistics.mean(f_all)
    f_std = statistics.stdev(f_all) if len(f_all) > 1 else 0.0

    per_class = {}
    for cid, (name, _) in heights.items():
        fs = [s["f_px"] for s in samples if s["clase"] == name]
        if fs:
            per_class[name] = round(statistics.median(fs), 2)

    print("\n" + "=" * 64)
    print("%-11s %8s %8s %8s %10s %8s" % ("clase", "d real", "h_px", "f_px", "d estim.", "error"))
    for s in samples:
        h_m = heights[s["cls_id"]][1]
        d_hat = distance_m(f_px, h_m, s["h_px"])
        err = 100.0 * (d_hat - s["d_m"]) / s["d_m"]
        s["d_estimada_m"] = round(d_hat, 3)
        s["error_pct"] = round(err, 1)
        print("%-11s %7.2fm %8.1f %8.1f %9.2fm %+7.1f%%"
              % (s["clase"], s["d_m"], s["h_px"], s["f_px"], d_hat, err))
    print("-" * 64)
    print("f_px = %.1f px  (desv. est. %.1f px = %.1f%%, %d puntos)"
          % (f_px, f_std, 100.0 * f_std / f_px, len(f_all)))
    for name, f in per_class.items():
        print("  f_px usando solo %-10s = %.1f" % (name, f))
    if f_px and f_std / f_px > 0.10:
        print("AVISO: la dispersión supera el 10%. Revisa las distancias medidas, que las "
              "cajas calcen bien con el objeto o mide más puntos.")
    print("=" * 64)
    return f_px, f_std, per_class


def save_yaml(path, args, img_w, img_h, f_px, f_std, per_class, heights, samples):
    data = {
        "formula": "d_m = f_px * alto_m / h_px   (h_px = alto de la caja YOLO en píxeles)",
        "fecha": datetime.datetime.now().isoformat(timespec="seconds"),
        "modelo": str(Path(args.model).resolve()),
        "camara": str(args.source),
        "resolucion": [img_w, img_h],
        "montada_en_robot": False,  # cámbialo a true cuando calibres ya montada
        "f_px": round(f_px, 2),
        "f_px_desv": round(f_std, 2),
        "f_px_por_clase": per_class,
        "alto_m": {name: h for _, (name, h) in sorted(heights.items())},
        "muestras": [{k: v for k, v in s.items() if k != "cls_id"} for s in samples],
    }
    with open(path, "w") as fh:
        yaml.safe_dump(data, fh, sort_keys=False, allow_unicode=True)
    print("Guardado en %s" % path)


def load_yaml(path):
    with open(path) as fh:
        return yaml.safe_load(fh)


# ----------------------------------------------------------------------------
# Modos
# ----------------------------------------------------------------------------
def infer(model, frame, args):
    return model.predict(frame, conf=args.conf, imgsz=args.imgsz,
                         device=args.device, verbose=False)[0]


def run_check(model, cap, img_w, img_h, heights, f_px, args, header=""):
    print("Modo verificación: mueve los objetos y compara con la huincha. 'q' para salir.")
    while True:
        ok, frame = cap.read()
        if not ok:
            print("Se perdió la cámara.")
            return
        boxes = best_box_per_class(infer(model, frame, args))
        vis = frame.copy()
        draw_boxes(vis, boxes, heights, img_w, img_h, f_px=f_px)
        put(vis, "VERIFICACION  f_px=%.1f  %s" % (f_px, header), 22, (0, 255, 255))
        put(vis, "q: salir", img_h - 12)
        cv2.imshow("calibracion", vis)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            return


def run_calibration(model, cap, img_w, img_h, heights, args):
    distances_cm = [float(x) for x in args.distances.split(",") if x.strip()]
    plan = [(cid, name, h, d / 100.0)
            for cid, (name, h) in sorted(heights.items())
            for d in distances_cm]

    samples = []           # resultados aceptados
    idx = 0                # punto actual del plan
    collecting = False
    hs, t0, msg = [], 0.0, ""

    print("Plan de calibración (%d puntos):" % len(plan))
    for cid, name, h, d in plan:
        print("  %-10s (alto %.0f cm) a %.0f cm" % (name, h * 100, d * 100))

    while idx < len(plan):
        cid, name, h_real, d_m = plan[idx]
        ok, frame = cap.read()
        if not ok:
            print("Se perdió la cámara.")
            break
        boxes = best_box_per_class(infer(model, frame, args))
        vis = frame.copy()
        draw_boxes(vis, boxes, heights, img_w, img_h)

        if collecting:
            box = boxes.get(cid)
            if box is not None and not is_truncated(box, img_w, img_h):
                hs.append(box[4] - box[2])
            elapsed = time.time() - t0
            put(vis, "Midiendo %s a %.0f cm: %d/%d" % (name, d_m * 100, len(hs), args.frames),
                22, (0, 255, 255))
            if len(hs) >= args.frames or elapsed > args.timeout:
                collecting = False
                if len(hs) >= max(5, args.frames // 2):
                    h_px = statistics.median(hs)
                    spread = (max(hs) - min(hs)) / h_px * 100.0
                    f = h_px * d_m / h_real
                    samples = [s for s in samples if not (s["clase"] == name and s["d_m"] == d_m)]
                    samples.append({"clase": name, "cls_id": cid, "d_m": d_m,
                                    "h_px": round(h_px, 2), "f_px": round(f, 2),
                                    "frames": len(hs)})
                    msg = "OK %s %.0fcm: h=%.1fpx f=%.1f (var. %.0f%%)" % (
                        name, d_m * 100, h_px, f, spread)
                    print(msg)
                    idx += 1
                else:
                    msg = "Pocas detecciones de %s (%d). Reacomoda y presiona ESPACIO." % (
                        name, len(hs))
                    print(msg)
        else:
            put(vis, "[%d/%d] Pon %s a %.0f cm y presiona ESPACIO"
                % (idx + 1, len(plan), name.upper(), d_m * 100), 22, (0, 255, 255))
            if cid not in boxes:
                put(vis, "No se detecta %s" % name, 48, (0, 0, 255))
            elif is_truncated(boxes[cid], img_w, img_h):
                put(vis, "%s cortado por el borde: alejalo o centralo" % name, 48, (0, 0, 255))

        if msg:
            put(vis, msg, img_h - 40, (200, 255, 200), 0.5)
        put(vis, "ESPACIO medir | n saltar | r repetir anterior | q terminar", img_h - 12,
            scale=0.5)
        cv2.imshow("calibracion", vis)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        if collecting:
            continue
        if key == ord(" "):
            collecting, hs, t0, msg = True, [], time.time(), ""
        elif key == ord("n"):
            msg = "Saltado %s %.0fcm" % (name, d_m * 100)
            idx += 1
        elif key == ord("r") and idx > 0:
            idx -= 1
            msg = "Repitiendo punto anterior"

    return samples


def main():
    args = parse_args()
    model = YOLO(args.model)
    heights = parse_heights(args.heights, model.names)
    cap, img_w, img_h = open_camera(args.source, args.width, args.height)
    print("Cámara %s a %dx%d | alturas: %s" % (
        args.source, img_w, img_h,
        ", ".join("%s=%.0fcm" % (n, h * 100) for n, h in heights.values())))

    try:
        if args.check:
            cal = load_yaml(args.out)
            if list(cal.get("resolucion", [img_w, img_h])) != [img_w, img_h]:
                print("AVISO: la calibración es de %s y la cámara está a %dx%d; "
                      "la distancia va a salir mal." % (cal["resolucion"], img_w, img_h))
            for cid, (name, _) in list(heights.items()):
                if name in cal.get("alto_m", {}):
                    heights[cid] = (name, float(cal["alto_m"][name]))
            run_check(model, cap, img_w, img_h, heights, float(cal["f_px"]), args,
                      header="(%s)" % Path(args.out).name)
            return

        samples = run_calibration(model, cap, img_w, img_h, heights, args)
        f_px, f_std, per_class = summarize(samples, heights)
        save_yaml(args.out, args, img_w, img_h, f_px, f_std, per_class, heights, samples)
        run_check(model, cap, img_w, img_h, heights, f_px, args)
    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
