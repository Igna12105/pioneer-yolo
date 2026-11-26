# Pioneer P3-DX: seguir y evadir objetos con YOLOv8

Proyecto Final de EL5206 (Universidad de Chile). Un Pioneer P3-DX controlado con ROS Melodic, con una webcam montada al frente, que detecta dos objetos con YOLOv8:

| Clase | Objeto | Alto real | Comportamiento |
|---|---|---|---|
| `follow_obj` (id 1) | taza | 10 cm | se acerca hasta quedar a 30 cm |
| `avoid_obj` (id 0) | naranja | 8 cm | retrocede si está a menos de 60 cm |

Si aparecen las dos a la vez, la naranja tiene prioridad cuando está cerca (ver [sección 9](#9-controlador-followavoid)).

---

## Índice

1. [Arquitectura](#1-arquitectura)
2. [Estructura del repositorio](#2-estructura-del-repositorio)
3. [Instalación](#3-instalación)
4. [Uso: correr el proyecto](#4-uso-correr-el-proyecto)
5. [Cámara](#5-cámara)
6. [Modelo YOLOv8](#6-modelo-yolov8)
7. [Calibración de distancia](#7-calibración-de-distancia)
8. [Puente UDP](#8-puente-udp)
9. [Controlador follow/avoid](#9-controlador-followavoid)
10. [Problemas frecuentes](#10-problemas-frecuentes)
11. [Pendientes](#11-pendientes)
12. [Créditos](#12-créditos)

---

## 1. Arquitectura

```
┌──────────── Notebook (Ubuntu, host) ─────────────┐        ┌────────── Docker: ROS Melodic (Python 2.7) ──────────┐
│                                                   │        │                                                       │
│  webcam USB (/dev/video2)                         │        │  udp_bridge_node.py ──► /yolo/detection               │
│        │                                          │  UDP   │                          │                            │
│        ▼                                          │ :5005  │                          ▼                            │
│  yolo/yolo_udp_sender.py  (conda, Python 3.11) ───┼───────►│  follow_avoid_controller.py ──► /RosAria/cmd_vel      │
│    YOLOv8n propio (yolo/model/best.pt)            │        │                                     │                 │
│                                                   │        │  RosAria ◄──────────────────────────┘                 │
└───────────────────────────────────────────────────┘        │     └──► Pioneer P3-DX (/dev/ttyUSB0)                 │
                                                             └───────────────────────────────────────────────────────┘
```

¿Por qué un puente UDP? ROS Melodic usa Python 2.7, y YOLOv8 (Ultralytics) necesita Python 3. YOLO corre en el host y le manda a ROS solo el resultado, un JSON chico por frame. El contenedor usa `network_mode: host`, así que basta con enviar a `127.0.0.1:5005`.

---

## 2. Estructura del repositorio

```text
pioneer-yolo/
├── README.md                  # esta guía
├── docs/DOCKER.md             # detalle del entorno Docker (instalación, GPU, espacio en disco)
├── Dockerfile                 # imagen base: ROS Melodic + AriaCoda + RosAria
├── Dockerfile.yolo            # variante opcional con YOLO dentro del contenedor
├── docker-compose*.yml        # dispositivos, pantalla, carpetas montadas, GPU
├── requirements-yolo.txt      # versiones de la variante YOLO del contenedor
├── .env.example               # plantilla de configuración (puerto del robot, cámara)
├── scripts/                   # run_container.sh, enter.sh, prepare_workspace.sh, check_host.sh
├── ros/yolo_bridge/           # paquete ROS del proyecto (se monta en el workspace)
│   ├── scripts/udp_bridge_node.py         # UDP -> /yolo/detection
│   ├── scripts/follow_avoid_controller.py # /yolo/detection -> /RosAria/cmd_vel
│   ├── config/follow_avoid.yaml           # calibración, distancias y velocidades
│   └── launch/                            # robot_yolo.launch (todo), udp_bridge.launch
└── yolo/                      # lado host (Python 3)
    ├── yolo_udp_sender.py     # cámara + YOLO -> UDP
    ├── udp_test_listener.py   # receptor de prueba sin ROS
    ├── calibrate_distance.py  # calibración de distancia
    ├── calibracion.yaml       # resultado de la calibración
    ├── requirements.txt       # entorno conda del host
    ├── train_yolov8_object_detection_on_custom_dataset.ipynb
    └── model/                 # best.pt (train3), data.yaml, args.yaml, métricas
```

Hay carpetas que no se suben al repo porque se generan solas: `pioneer_ws/` (lo clona `prepare_workspace.sh`), `data/`, `models/` y `.env`.

---

## 3. Instalación

Probado en Ubuntu 24.04 (sesión Wayland), GTX 1650 y Docker.

### 3.1 Contenedor ROS
```bash
git clone <url-del-repo> pioneer-yolo
cd pioneer-yolo
chmod +x scripts/*.sh
./scripts/check_host.sh          # revisa requisitos (no cambia nada)
./scripts/run_container.sh       # la primera vez clona pioneer_ws y rosaria y construye la imagen (varios minutos)
```
Con el contenedor arriba se compila el workspace (incluye `yolo_bridge`):
```bash
./scripts/enter.sh
source /opt/ros/melodic/setup.bash && catkin_make && source devel/setup.bash
rospack find yolo_bridge         # /root/pioneer_ws/src/yolo_bridge
```
Detalles de Docker, GPU, espacio en disco y la variante con YOLO dentro del contenedor: [`docs/DOCKER.md`](docs/DOCKER.md).

### 3.2 Entorno YOLO en el host
```bash
conda create -n YOLOv8 python=3.11 -y
conda activate YOLOv8
pip install -r yolo/requirements.txt
python -c "import torch; print(torch.cuda.is_available())"   # True si usa la GPU
```

### 3.3 Permisos
```bash
sudo usermod -aG video $USER     # cámara; después cerrar sesión y volver a entrar
sudo apt remove brltty           # solo si /dev/ttyUSB0 aparece y desaparece
```

---

## 4. Uso: correr el proyecto

Antes de empezar: conecta el robot por USB y enciéndelo (`ls -l /dev/ttyUSB*`). Para las primeras pruebas, déjalo con las ruedas en el aire.

**Host: levantar el contenedor** (si no está corriendo)
```bash
./scripts/run_container.sh
```

**Terminal 1, contenedor: RosAria, puente y controlador**
```bash
./scripts/enter.sh
source /opt/ros/melodic/setup.bash && source devel/setup.bash
roslaunch yolo_bridge robot_yolo.launch dry_run:=true   # en seco: el robot NO se mueve (cmd_vel va a /cmd_vel_dry)
roslaunch yolo_bridge robot_yolo.launch                 # con el robot
#   ajustes: d_target:=0.35 d_safe:=0.8 v_max:=0.06 w_max:=0.2
```
El launch levanta su propio `roscore`. Antes de lanzarlo, cierra cualquier `roscore`, RosAria o `udp_bridge` que haya quedado abierto.

**Terminal 2, contenedor: monitor**
```bash
./scripts/enter.sh
source devel/setup.bash
rostopic echo /follow_avoid/state      # FOLLOW / AVOID / AVOID_OK / LOST / STALE
# también: /cmd_vel_dry, /RosAria/cmd_vel, /follow_avoid/distance, /yolo/detection
```

**Terminal 3, host: emisor YOLO**
```bash
conda activate YOLOv8
cd yolo
python yolo_udp_sender.py --model model/best.pt --source 2 \
  --host 127.0.0.1 --port 5005 --conf 0.5 --device 0 --width 640 --height 480 --show
```

**Qué tiene que pasar en la prueba en seco:**

| Situación | Comando esperado |
|---|---|
| Taza a la derecha de la imagen | `angular.z` negativo |
| Taza lejos y centrada | `linear.x` positivo, hasta 0,10 |
| Taza a unos 30 cm | todo en cero |
| Naranja a menos de 60 cm | `linear.x` negativo, hasta -0,08 |
| Taza lejos + naranja a unos 70 cm | estado `FOLLOW_BLOQUEADO`, `linear.x` = 0 |
| Taza lejos + naranja a más de 1 m | estado `FOLLOW`, `linear.x` positivo |
| Sin objetos | estado `LOST`, todo en cero |

**Para detenerlo:** Ctrl+C en el emisor hace que el robot frene en 0,5 s. Ctrl+C en el launch apaga todo y manda velocidad cero. Igual ten a mano el botón de motores del Pioneer.

Para mover el robot a mano (prueba de RosAria):
```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/RosAria/cmd_vel
```

---

## 5. Cámara

- Webcam UVC (`USB_Camera`). Ubuntu la reconoce sin instalar drivers.
- `v4l2-ctl --list-devices` (paquete `v4l-utils`): `/dev/video0` es la cámara integrada del notebook y **`/dev/video2` es la del robot → `--source 2`**. Los nodos impares (`video1`, `video3`) son de metadatos y no entregan imagen.
- Formatos: MJPG llega a 30 fps hasta 1080p; YUYV baja a 8 fps en 720p. El proyecto usa 640x480.
- Solo un programa a la vez puede usar la cámara.
- Ruta estable, por si el índice cambia al reconectar: `ls -l /dev/v4l/by-id/`.
- Prueba rápida sin YOLO: `ffplay -f v4l2 -input_format mjpeg /dev/video2` (requiere `sudo apt install ffmpeg`).

---

## 6. Modelo YOLOv8

- Dataset en Roboflow: workspace `proyecto-lab-de-robotica`, proyecto `pioneer-robot-object-detection`, **versión 4**, exportado en formato YOLOv8. El preprocesamiento es "Fit (black edges)" a 640x640, que mantiene la proporción de la imagen.
- Entrenamiento local (`yolo/train_yolov8_object_detection_on_custom_dataset.ipynb`): `yolov8n.pt` como base, 80 épocas, batch 8, imgsz 640 y `amp=False`, porque las GTX 16xx dan NaN con AMP.
- Pesos: `yolo/model/best.pt` (corrida `train3`). Validación: precisión 0.997, recall 0.980, mAP50 0.989, mAP50-95 0.970. Las curvas y la matriz de confusión están en `yolo/model/metrics/`.
- Para descargar el dataset desde el notebook se necesita una API key de Roboflow **propia**. No va en el código; se define como variable de entorno antes de abrir el notebook:
  ```bash
  export ROBOFLOW_API_KEY=...
  jupyter notebook yolo/train_yolov8_object_detection_on_custom_dataset.ipynb
  ```

Prueba rápida del detector, sin ROS:
```bash
cd yolo && yolo predict model=model/best.pt source=2 show=True conf=0.5
```

---

## 7. Calibración de distancia

Se usa el modelo estenopeico (pinhole): `d = f_px · alto_real / h_px`, donde `h_px` es el alto de la caja de YOLO en píxeles.

- `yolo/calibrate_distance.py` pide poner cada objeto a distancias conocidas (por defecto 30, 50, 75, 100 y 150 cm, medidas desde el lente hasta el frente del objeto). En cada punto promedia 30 frames y calcula `f_px`.
- Resultado actual (`yolo/calibracion.yaml`, cámara montada en el robot, 640x480): `f_px` = 302,1 para la taza y 321,9 para la naranja. El error está entre -4 % y +8 % en el rango de 0,3 a 1,5 m.
- La cámara está al frente del robot, así que la distancia desde la cámara es la distancia desde el frente del robot.

```bash
cd yolo
python calibrate_distance.py            # calibrar (ESPACIO medir, n saltar, r repetir, q terminar)
python calibrate_distance.py --check    # ver la distancia estimada en vivo
```
Si se mueve la cámara o cambia la resolución, hay que recalibrar y copiar los nuevos `f_px` a `ros/yolo_bridge/config/follow_avoid.yaml`.

---

## 8. Puente UDP

**Emisor (`yolo/yolo_udp_sender.py`, host):** envía un JSON por frame, también cuando no detecta nada (`det=0`), para que el receptor distinga "no veo el objeto" de "se cayó el emisor". Los campos `det`…`cls` son la detección de mayor confianza; `follow` y `avoid` traen la mejor caja de **cada** clase (con `det=0` si esa clase no está en el frame).
```json
{"seq": 123, "t": 1795021000.1, "det": 1, "cx": -0.31, "cy": 0.13,
 "w": 0.10, "h": 0.13, "area": 0.013, "conf": 0.94, "cls": 0,
 "follow": {"det": 1, "cx": 0.25, "cy": 0.05, "w": 0.12, "h": 0.21, "conf": 0.91},
 "avoid":  {"det": 1, "cx": -0.31, "cy": 0.13, "w": 0.10, "h": 0.13, "conf": 0.94}}
```

| Campo | Significado |
|---|---|
| `cx`, `cy` | centro de la caja normalizado en [-1, 1]; `cx` positivo = derecha, `cy` positivo = abajo |
| `w`, `h` | ancho y alto de la caja como fracción de la imagen |
| `area` | `w·h` |
| `cls` | id de clase (-1 si no hay detección) |

Opciones: `--model`, `--source`, `--host`, `--port`, `--conf`, `--imgsz`, `--device`, `--target <nombre|id>`, `--follow-class` (por defecto `follow_obj`), `--avoid-class` (por defecto `avoid_obj`), `--width`, `--height`, `--show`.

**Receptor (`ros/yolo_bridge/scripts/udp_bridge_node.py`, contenedor):** publica `/yolo/detected` (`Bool`) y `/yolo/detection` (`Float32MultiArray`) con 16 valores:
```
[det, cx, cy, w, h, area, conf, cls,    # mejor detección (índices 0-7)
 f_det, f_cx, f_h, f_conf,              # taza    (8-11)
 a_det, a_cx, a_h, a_conf]              # naranja (12-15)
```
Si el emisor es una versión antigua sin `follow`/`avoid`, publica solo los 8 primeros, y el controlador también funciona con eso. Si pasan más de 0,3 s sin paquetes, publica todo en cero a 10 Hz.

**Probar solo el emisor, sin Docker** (con el contenedor apagado, porque si no el puerto queda ocupado):
```bash
python3 yolo/udp_test_listener.py --port 5005
```

---

## 9. Controlador follow/avoid

`ros/yolo_bridge/scripts/follow_avoid_controller.py` (Python 2.7) lee `/yolo/detection` y publica en `/RosAria/cmd_vel`. Calcula la distancia como `d = f_px[clase] · alto[clase] / (h · 480)`.

Calcula una distancia por clase, cada una con su propio filtro, y decide con prioridades (la primera que se cumple manda):

| Prioridad | Estado | Situación | Acción |
|---|---|---|---|
| 1 | `AVOID` | Naranja a menos de `d_safe` = 0,60 m (con o sin taza) | Retrocede manteniéndola centrada hasta pasar 0,65 m. Ignora la taza. |
| 2 | `FOLLOW_BLOQUEADO` | Taza visible y naranja a menos de `d_block` = 0,80 m | Gira hacia la taza pero **no avanza** (sí puede retroceder). Se desbloquea sobre 0,85 m. |
| 3 | `FOLLOW` | Taza visible | La centra y avanza hasta `d_target` = 0,30 m (±3 cm). Si está muy descentrada (`|cx|` ≥ 0,5), primero gira. Si queda a menos de 0,27 m, retrocede. |
| 4 | `AVOID_OK` | Solo naranja, más lejos de 0,60 m | Se queda quieto, girando para mantenerla centrada. |
| 5 | `LOST` | Nada, o confianza < 0,5 | Frena. |
| — | `STALE` | `/yolo/detection` no llega por 0,5 s | Frena. |

Sin memoria: si la naranja sale del encuadre (por ejemplo al girar hacia la taza), el controlador deja de considerarla.

- **Modo lento (actual):** 0,10 m/s hacia adelante, 0,08 m/s hacia atrás y 0,30 rad/s de giro. Las rampas de aceleración son suaves y el frenado es rápido. Al apagarse publica velocidad cero.
- **Parámetros:** en `ros/yolo_bridge/config/follow_avoid.yaml`. `d_target`, `d_safe`, `v_max` y `w_max` también se pueden cambiar como argumentos del launch.
- **Para depurar:** `/follow_avoid/state`, `/follow_avoid/distance` (objeto que manda), `/follow_avoid/distance_follow` y `/follow_avoid/distance_avoid`.
- Como `ros/yolo_bridge` está montado dentro del contenedor, los cambios se ven al relanzar, sin reconstruir la imagen. Solo hay que hacer `catkin_make` si se agregan archivos nuevos.

---

## 10. Créditos

- Entorno Docker basado en el bundle `pioneer_p3dx_docker_bundle.zip` de Samuel Bugueño.
- Workspace ROS del Laboratorio de Visión Computacional, DIE, Universidad de Chile: [`Uchile-Lab-Vision-Computacional/pioneer_ws`](https://github.com/Uchile-Lab-Vision-Computacional/pioneer_ws).
- [RosAria](https://github.com/amor-ros-pkg/rosaria) y [AriaCoda](https://github.com/reedhedges/AriaCoda).
- [Ultralytics YOLOv8](https://github.com/ultralytics/ultralytics) y el notebook base de [Roboflow](https://github.com/roboflow/notebooks).
