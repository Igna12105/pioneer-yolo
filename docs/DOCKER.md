# Entorno Docker: Pioneer P3-DX + ROS Melodic (+ YOLO opcional)

Detalle del entorno Docker del proyecto. La guía para correr el proyecto está en el
[README principal](../README.md). Todos los comandos se ejecutan desde la raíz del repositorio.

Probado en: Ubuntu 24.04, sesión Wayland, GTX 1650 (driver 595), Docker con almacenamiento containerd.
Basado en el bundle `pioneer_p3dx_docker_bundle.zip` (Samuel Bugueño) y en el repositorio
[`pioneer_ws`](https://github.com/Uchile-Lab-Vision-Computacional/pioneer_ws).

> En el flujo actual del proyecto YOLO corre **en el host** (entorno conda, ver README principal) y
> el contenedor se usa en su variante base (`./scripts/run_container.sh`, contenedor `pioneer_melodic`).
> La variante `--yolo` queda disponible para quien prefiera correr YOLO dentro del contenedor.

---

## Índice

1. [Conceptos básicos](#1-conceptos-básicos)
2. [Estructura de la carpeta](#2-estructura-de-la-carpeta)
3. [Primera instalación](#3-primera-instalación)
4. [Uso diario](#4-uso-diario)
5. [Pruebas del entorno](#5-pruebas-del-entorno)
6. [Probar el robot](#6-probar-el-robot)
7. [Usar YOLO](#7-usar-yolo)
8. [Entrenar y cargar el modelo propio](#8-entrenar-y-cargar-el-modelo-propio)
9. [Flujo del proyecto](#9-flujo-del-proyecto)
10. [Espacio en disco](#10-espacio-en-disco)
11. [Problemas frecuentes](#11-problemas-frecuentes)
12. [Cambios respecto al bundle original](#12-cambios-respecto-al-bundle-original)

---

## 1. Conceptos básicos

**Imagen y contenedor**

- La **imagen** es el "molde": trae ROS, AriaCoda, RosAria, YOLO, etc. Se construye una vez, y solo
  otra vez si cambias un `Dockerfile` o `requirements-yolo.txt`.
- El **contenedor** es la "instancia" que corre a partir de la imagen. Se inicia, se detiene y se
  vuelve a iniciar sin tocar la imagen.

Hay dos imágenes:

| Imagen | Contenido | Contenedor |
|---|---|---|
| `pioneer-melodic:base` | ROS Melodic + AriaCoda + RosAria + `usb_cam` | `pioneer_melodic` |
| `pioneer-melodic:yolo` | La base + Python 3.10 con PyTorch (CUDA 12.1) y ultralytics | `pioneer_melodic_yolo` |

La imagen YOLO se construye encima de la base: **no borres la base**.

**Dos Pythons dentro del contenedor**

ROS Melodic usa Python 2.7 y YOLO necesita Python 3. Por eso conviven dos intérpretes:

| | Intérprete | Se usa para |
|---|---|---|
| ROS | `python` (2.7 del sistema) | `rospy`, RosAria, nodo puente, nodo de control |
| YOLO | `yolo-python` (3.10, en `/opt/venvs/yolo`) | ultralytics, PyTorch, OpenCV |

- Los scripts de YOLO se ejecutan con `yolo-python script.py`. No uses `source .../activate`:
  el entorno lo crea micromamba y no trae ese archivo.
- No cambies el `PATH` en las terminales de ROS. Los nodos usan `#!/usr/bin/env python` y tomarían
  el Python 3.10, que no tiene `rospy`.
- Por esto mismo, el script de YOLO no puede publicar en ROS directamente: se comunica con ROS por
  UDP (sección 9).

**Carpetas compartidas con el notebook**

Estas carpetas viven en el notebook y el contenedor las ve en vivo. Lo que edites ahí no requiere
reconstruir nada.

| En el notebook | En el contenedor | Para qué |
|---|---|---|
| `pioneer_ws/` | `/root/pioneer_ws` | Workspace catkin (RosAria y sus paquetes) |
| `ros/yolo_bridge/` | `/root/pioneer_ws/src/yolo_bridge` | Paquete del proyecto (puente UDP y controlador) |
| `data/` | `/root/data` | Scripts propios, pesos `best.pt`, videos, resultados |
| `models/` | `/root/.config/Ultralytics` | Configuración de ultralytics |

---

## 2. Estructura

La estructura completa del repositorio está en el [README principal](../README.md#2-estructura-del-repositorio).
Archivos del entorno Docker:

```text
├── Dockerfile                 # imagen base (ROS Melodic + AriaCoda)
├── Dockerfile.yolo            # variante YOLO
├── docker-compose.yml         # contenedor base: dispositivos, pantalla, carpetas
├── docker-compose.yolo.yml    # cambia la imagen a la variante YOLO
├── docker-compose.gpu.yml     # agrega la GPU NVIDIA
├── requirements-yolo.txt      # versiones exactas de las librerías de YOLO (variante YOLO)
├── .env.example               # plantilla de configuración (puertos, cámara)
├── .env                       # tu configuración (se crea sola la primera vez; no se sube al repo)
├── scripts/
│   ├── check_host.sh          # revisa requisitos del notebook (no cambia nada)
│   ├── run_container.sh       # construye si hace falta y crea el contenedor
│   ├── enter.sh               # abre una terminal dentro del contenedor
│   ├── prepare_workspace.sh   # clona pioneer_ws y rosaria (lo llama run_container.sh)
│   └── container_entrypoint.sh
├── pioneer_ws/                # workspace (se crea la primera vez; no se sube al repo)
├── data/                      # archivos propios para la variante YOLO (no se sube)
└── models/                    # configuración de ultralytics (no se sube)
```

---

## 3. Primera instalación

Solo se hace una vez (o si borras las imágenes).

### 3.1 Revisar el notebook

```bash
cd pioneer-yolo
chmod +x scripts/*.sh
df -h /                       # necesitas ~15 GB libres para construir
./scripts/check_host.sh
```

Corrige lo que aparezca como `[FALTA]`. Los `[AVISO]` de GPU solo importan si usarás `--gpu`.

### 3.2 GPU (solo si falta nvidia-container-toolkit)

```bash
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list \
  | sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' \
  | sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
sudo apt update && sudo apt install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
docker run --rm --gpus all ubuntu nvidia-smi    # debe mostrar la GTX 1650
```

### 3.3 Configuración

```bash
cp .env.example .env
```

Revisa en `.env`:

- `PIONEER_DEVICE`: puerto del robot (normalmente `/dev/ttyUSB0`).
- `CAMERA_DEVICE`: cámara a usar. Por defecto `/dev/video2` (webcam USB del robot); la del notebook
  es `/dev/video0`. Revísalo con `v4l2-ctl --list-devices` dentro del contenedor.

### 3.4 Construir y crear el contenedor

Elige **una** opción:

```bash
./scripts/run_container.sh                # solo ROS + RosAria (lo que usa el proyecto)
./scripts/run_container.sh --yolo --gpu   # además YOLO con GPU dentro del contenedor
./scripts/run_container.sh --yolo         # además YOLO en CPU dentro del contenedor
```

- La primera vez clona `pioneer_ws` y `rosaria`, construye las imágenes (varios minutos y varios GB)
  y compila el workspace con `catkin_make`.
- El aviso `/dev/ttyUSB0 no existe` es normal si el robot no está conectado. El contenedor arranca
  igual y detecta el robot cuando lo enchufes.

Al terminar, recupera espacio borrando la caché del build (no borra las imágenes):

```bash
docker builder prune -af
```

---

## 4. Uso diario

Después de la primera instalación **no hace falta `run_container.sh`**.

```bash
docker start pioneer_melodic         # iniciar (si está detenido); con la variante YOLO: pioneer_melodic_yolo
./scripts/enter.sh                    # abrir una terminal dentro (repite en cada terminal que necesites)
docker stop pioneer_melodic          # detener al terminar
```

- Para ver si existe y en qué estado está: `docker ps -a`.
- El mensaje `groups: cannot find name for group ID 992` al entrar es inofensivo.

**Cuándo sí usar `run_container.sh`**

| Situación | Comando |
|---|---|
| Cambiar de variante (base, YOLO, GPU) | `./scripts/run_container.sh --yolo --gpu` (o la variante que quieras) |
| Borraste el contenedor | igual que arriba |
| Cambiaste `Dockerfile`, `Dockerfile.yolo` o `requirements-yolo.txt` | agregar `--build`, p. ej. `./scripts/run_container.sh --yolo --gpu --build` |

Sin `--build`, el script solo construye la imagen que falte.

**Ojo:** `run_container.sh` borra y recrea el contenedor. Si tienes terminales abiertas dentro,
se cierran. Lo que está en `pioneer_ws/`, `ros/`, `data/` y `models/` no se pierde; lo que hayas instalado a
mano dentro del contenedor (`apt install`, `pip install`) sí.

---

## 5. Pruebas del entorno

Con el contenedor corriendo, en terminales dentro del contenedor (`./scripts/enter.sh`).

**5.1 ROS master (siempre primero)**

Todos los programas de ROS (RViz, RosAria, rqt, tus nodos) necesitan el master corriendo.

```bash
roscore
```

Deja esa terminal abierta. Si ves `Could not contact ROS master at [http://127.0.0.1:11311]`, falta
este paso.

**5.2 Workspace**

```bash
ls /root/pioneer_ws/devel/setup.bash && rospack find rosaria
```

**5.3 Pantalla**

```bash
xeyes     # ventana con ojos; Ctrl+C para cerrar
rviz      # requiere roscore
```

**5.4 GPU**

```bash
nvidia-smi
yolo-python -c "import torch; print(torch.__version__, '| GPU:', torch.cuda.is_available())"
```

Esperado: `2.2.2+cu121 | GPU: True`.

**5.5 Cámara**

```bash
v4l2-ctl --list-devices
```

Cada cámara aparece con dos dispositivos (p. ej. `/dev/video0` y `/dev/video1`); el primero es la
imagen y el segundo son metadatos.

Ver la cámara en ROS (requiere roscore):

```bash
rosrun usb_cam usb_cam_node _video_device:=$CAMERA_DEVICE
# en otra terminal:
rqt_image_view            # elegir /usb_cam/image_raw
```

**Una cámara solo la puede usar un programa a la vez**: cierra `usb_cam` antes de usar YOLO con la
cámara, y viceversa.

---

## 6. Probar el robot

1. Conecta el cable USB-serial del Pioneer y enciéndelo.
2. Verifica el puerto (dentro o fuera del contenedor):
   ```bash
   ls -l /dev/ttyUSB*
   ```
3. Con `roscore` corriendo, en otra terminal:
   ```bash
   rosrun rosaria RosAria _port:=$PIONEER_DEVICE
   ```
   Debe conectarse sin errores.
4. Revisa los tópicos:
   ```bash
   rostopic list | grep RosAria     # /RosAria/cmd_vel, /RosAria/pose, /RosAria/sonar, ...
   rostopic echo /RosAria/pose      # odometría
   ```
5. Mover con el teclado (velocidades bajas, robot con espacio libre alrededor):
   ```bash
   rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/RosAria/cmd_vel
   ```

---

## 7. Usar YOLO

Siempre con `yolo-python`. Los scripts conviene dejarlos en `data/` (en el contenedor, `/root/data`).

**Prueba rápida con la cámara**

```bash
cd /root/data
yolo-python -c "from ultralytics import YOLO; YOLO('yolov8n.pt').predict(source=0, show=True, conf=0.4)"
```

- `source=0` es `/dev/video0`; para otra cámara usa `source=2`, etc.
- La primera vez descarga `yolov8n.pt` (~6 MB) en la carpeta actual.
- Con la GTX 1650, cada imagen tarda unos 7 a 19 ms.
- Para probar sin modelo propio sirve la clase `cup` de COCO (id 41).

**Esqueleto de un script propio** (`data/mi_yolo.py`)

```python
from ultralytics import YOLO
import cv2

model = YOLO('/root/data/yolov8n.pt')        # luego: '/root/data/best.pt'
cap = cv2.VideoCapture(0)

while True:
    ok, frame = cap.read()
    if not ok:
        break
    r = model.predict(frame, conf=0.4, verbose=False)[0]
    for box in r.boxes:
        cls = model.names[int(box.cls)]
        x1, y1, x2, y2 = box.xyxy[0].tolist()
        cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
        area = (x2 - x1) * (y2 - y1)
        print(cls, round(cx), round(area), float(box.conf))
    cv2.imshow('yolo', r.plot())
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
```

Se ejecuta con `yolo-python /root/data/mi_yolo.py` y se cierra con `q` en la ventana.

---

## 8. Entrenar y cargar el modelo propio

1. **Dataset en Roboflow** (taza y naranja): exportar en formato **YOLOv8**. Ese formato sirve
   también para YOLO11.
> El modelo del proyecto (`yolo/model/best.pt`) se entrenó en el host con ultralytics 8.2.103
> (ver README principal). Esta sección aplica si se entrena para usar dentro del contenedor.

2. **Entrenar en Colab con la misma versión de ultralytics que tiene la imagen**:
   ```python
   !pip install ultralytics==8.3.0
   from ultralytics import YOLO
   model = YOLO('yolov8n.pt')            # o 'yolo11n.pt'
   model.train(data='/content/<dataset>/data.yaml', epochs=100, imgsz=640)
   ```
   Si entrenan con una versión más nueva, los pesos pueden no cargar en la 8.3.0. Para cambiar la
   versión de la imagen hay que editar `requirements-yolo.txt` y reconstruir solo la parte YOLO con
   `--build`.
3. **Copiar los pesos**: descargar `runs/detect/train/weights/best.pt` y dejarlo en
   `pioneer_melodic_docker/data/best.pt`.
4. **Usarlo**: `YOLO('/root/data/best.pt')`. Los nombres de las clases quedan en `model.names`.

Cambiar de modelo no requiere reconstruir ni reiniciar el contenedor: basta reemplazar el archivo.

---

## 9. Flujo del proyecto

El flujo implementado (emisor YOLO en el host → puente UDP → controlador → RosAria) y los comandos
para correrlo están en el [README principal](../README.md#4-uso-correr-el-proyecto).

---

## 10. Espacio en disco

La variante YOLO pesa varios GB (PyTorch con CUDA), y con el almacenamiento containerd cada imagen
queda guardada comprimida y desempaquetada.

**Ver espacio** (solo muestra):

```bash
df -h /
docker system df        # columna RECLAIMABLE = lo que se podría liberar
docker images
docker ps -a
```

**Liberar espacio de forma segura**

```bash
docker builder prune -af     # caché de construcción: suele ser lo que más pesa; no borra imágenes
docker image prune -f        # capas huérfanas de builds fallidos
```

**No uses** `docker system prune -a` ni `docker image prune -a` sin revisar antes `docker images`:
borran todas las imágenes que no tengan un contenedor corriendo, incluidas las de otros proyectos y
`pioneer-melodic:base`.

Antes de un `--build`, confirma que tengas al menos 15 GB libres.

---

## 11. Problemas frecuentes

| Síntoma | Causa / solución |
|---|---|
| `Could not contact ROS master` | Falta `roscore` en otra terminal |
| `No hay contenedor del Pioneer corriendo` | `docker start pioneer_melodic` o `./scripts/run_container.sh` |
| `no space left on device` durante el build | Ver sección 10; luego relanzar `run_container.sh` (sin `--build` si la imagen alcanzó a crearse) |
| `Unable to open display` | Ejecutar los scripts desde una terminal de la sesión gráfica (no por SSH). Respaldo en el host: `xhost +SI:localuser:root`. Último recurso: iniciar sesión en "Ubuntu on Xorg" |
| RViz muy lento | En el host debe existir `/dev/dri/renderD*`; dentro, `glxinfo -B` no debería decir `llvmpipe` |
| YOLO no abre la cámara | Otro programa la está usando (p. ej. `usb_cam`), o el índice `source=` no corresponde |
| `torch.cuda.is_available()` es `False` | Contenedor creado sin `--gpu`, o falta nvidia-container-toolkit (sección 3.2) |
| `/dev/ttyUSB0` aparece y desaparece | En el host: `sudo apt remove brltty` |
| RosAria no conecta | Robot apagado, puerto equivocado en `.env` o cable mal conectado; revisar `ls -l /dev/ttyUSB*` |
| Los pesos entrenados no cargan | Se entrenaron con otra versión de ultralytics; ver sección 8 |
| `apt-get update` falla con `EXPKEYSIG` en un `--build` | Expiró otra vez la llave de `snapshots.ros.org`; hay que actualizarla en el `Dockerfile` |

---

## 12. Cambios respecto al bundle original

| Bundle original | Cambio |
|---|---|
| Forzaba los repos de Ubuntu a `old-releases`, pero Bionic sigue en `archive.ubuntu.com` | Usa `archive` y solo pasa a `old-releases` si falla |
| La llave de `snapshots.ros.org` ha expirado varias veces | Se refresca desde el keyserver antes de `apt-get update` |
| AriaCoda desde `master` y `make install`, que exige `doxygen` | Commit fijo; `make install-default install-utils`; se verifica `Aria.h` y `libAria.so` |
| Sin webcam en el contenedor | Se monta `/dev` con permisos para serial, video y DRI; se agrega `usb_cam` |
| No arrancaba sin el robot conectado | Arranca igual y detecta el robot y la cámara al conectarlos |
| YOLO solo en CPU | Opción `--gpu` (PyTorch 2.2.2 con CUDA 12.1) |
| Dependencias de YOLO sin fijar | `requirements-yolo.txt` con versiones exactas compatibles con Ubuntu 18.04 |
| Wayland | Qt forzado a X11 (XWayland) y detección de la cookie de XWayland |
| Reconstruía en cada arranque | Solo construye si falta la imagen o con `--build` |
| — | Scripts nuevos `check_host.sh` y `enter.sh` |
| — | `ros/yolo_bridge` montado en el workspace; cámara por defecto `/dev/video2` |
