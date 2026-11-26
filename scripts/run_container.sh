#!/usr/bin/env bash
# Uso: ./scripts/run_container.sh [--yolo] [--gpu] [--build]
#   (sin flags)     variante base: ROS Melodic + RosAria
#   --yolo          variante con YOLOv8 (Python 3.10 aislado)
#   --yolo --gpu    YOLOv8 usando la GPU NVIDIA
#   --build         fuerza reconstruir la imagen (si cambiaste un Dockerfile o
#                   requirements-yolo.txt). Sin esta opción solo se construye
#                   la imagen que falte.
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${ROOT_DIR}"

YOLO=0
GPU=0
BUILD=0
for arg in "$@"; do
  case "${arg}" in
    --yolo) YOLO=1 ;;
    --gpu)  GPU=1 ;;
    --build) BUILD=1 ;;
    *) echo "Uso: $0 [--yolo] [--gpu] [--build]" >&2; exit 2 ;;
  esac
done
if [[ "${GPU}" -eq 1 && "${YOLO}" -eq 0 ]]; then
  echo "--gpu solo tiene sentido con --yolo (la imagen base no usa CUDA)." >&2
  exit 2
fi

if [[ ! -f .env ]]; then
  cp .env.example .env
  echo "Se creó .env desde .env.example."
fi
set -a
# shellcheck disable=SC1091
source .env
set +a

# --- Pantalla (X11 / XWayland) ---------------------------------------------
if [[ -z "${DISPLAY:-}" ]]; then
  echo "AVISO: DISPLAY está vacío. El contenedor arranca, pero no se podrán abrir ventanas (RViz, rqt)."
fi
if ! command -v xauth >/dev/null 2>&1; then
  echo "ERROR: falta xauth en el host. Instálalo con: sudo apt install xauth" >&2
  exit 1
fi
# En GNOME Wayland la cookie de XWayland está en /run/user/$UID/.mutter-Xwaylandauth.*
if [[ -z "${XAUTHORITY:-}" ]]; then
  XWAUTH="$(ls -1t /run/user/"$(id -u)"/.mutter-Xwaylandauth.* 2>/dev/null | head -n1 || true)"
  if [[ -n "${XWAUTH}" ]]; then export XAUTHORITY="${XWAUTH}"; fi
fi

# Si una ejecución anterior dejó /tmp/.docker.xauth como directorio, se elimina.
if [[ -d /tmp/.docker.xauth ]]; then rmdir /tmp/.docker.xauth 2>/dev/null || sudo rm -rf /tmp/.docker.xauth; fi
: > /tmp/.docker.xauth
chmod 600 /tmp/.docker.xauth
if [[ -n "${DISPLAY:-}" ]]; then
  xauth nlist "${DISPLAY}" 2>/dev/null | sed -e 's/^..../ffff/' | xauth -f /tmp/.docker.xauth nmerge - || true
  if [[ ! -s /tmp/.docker.xauth ]]; then
    echo "AVISO: no se encontró cookie X11; se habilita acceso local con xhost (solo usuarios locales)."
    xhost +SI:localuser:root >/dev/null 2>&1 || true
  fi
fi

# --- Dispositivos (solo avisos: el contenedor arranca igual) ----------------
DEVICE="${PIONEER_DEVICE:-/dev/ttyUSB0}"
CAMERA="${CAMERA_DEVICE:-/dev/video0}"
if [[ ! -e "${DEVICE}" ]]; then
  echo "AVISO: ${DEVICE} no existe (Pioneer no conectado). Puedes conectarlo después; el contenedor lo verá."
fi
if [[ ! -e "${CAMERA}" ]]; then
  echo "AVISO: ${CAMERA} no existe (webcam no detectada)."
fi

# --- Workspace --------------------------------------------------------------
WS_DIR="${PIONEER_WS_HOST:-./pioneer_ws}"
if [[ "${WS_DIR}" != /* ]]; then WS_DIR="${ROOT_DIR}/${WS_DIR}"; fi
if [[ ! -d "${WS_DIR}/src/rosaria" ]]; then
  PIONEER_WS_HOST="${WS_DIR}" ./scripts/prepare_workspace.sh
fi
mkdir -p data models

# --- GPU --------------------------------------------------------------------
if [[ "${GPU}" -eq 1 ]]; then
  if ! command -v nvidia-smi >/dev/null 2>&1 || ! nvidia-smi >/dev/null 2>&1; then
    echo "ERROR: nvidia-smi no funciona en el host; revisa el driver NVIDIA." >&2
    exit 1
  fi
  if ! docker info 2>/dev/null | grep -qi nvidia; then
    echo "ERROR: Docker no tiene el runtime NVIDIA. Instala/configura nvidia-container-toolkit (ver README, sección GPU)." >&2
    exit 1
  fi
fi

# --- Construir (solo si hace falta) e iniciar -------------------------------
image_exists() { docker image inspect "$1" >/dev/null 2>&1; }

COMPOSE=(docker compose -f docker-compose.yml)
if [[ "${BUILD}" -eq 1 ]] || ! image_exists pioneer-melodic:base; then
  echo "Construyendo imagen base..."
  docker compose -f docker-compose.yml build pioneer
fi
if [[ "${YOLO}" -eq 1 ]]; then
  COMPOSE+=(-f docker-compose.yolo.yml)
  if [[ "${BUILD}" -eq 1 ]] || ! image_exists pioneer-melodic:yolo; then
    echo "Construyendo imagen YOLO..."
    "${COMPOSE[@]}" build pioneer
  fi
fi
if [[ "${GPU}" -eq 1 ]]; then COMPOSE+=(-f docker-compose.gpu.yml); fi

# Solo una variante a la vez (comparten el puerto del robot).
docker rm -f pioneer_melodic pioneer_melodic_yolo >/dev/null 2>&1 || true
"${COMPOSE[@]}" up -d --no-build

printf '\nContenedor iniciado. Entra con:\n  ./scripts/enter.sh\n'
printf 'Para detenerlo:\n  %s down\n' "${COMPOSE[*]}"
