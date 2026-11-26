#!/usr/bin/env bash
# Revisa que el notebook (Ubuntu 24, Wayland, GTX 1650) tenga lo necesario.
# No modifica nada: solo informa. Uso: ./scripts/check_host.sh
ok()   { printf '  \e[32m[OK]\e[0m   %s\n' "$*"; }
warn() { printf '  \e[33m[AVISO]\e[0m %s\n' "$*"; }
bad()  { printf '  \e[31m[FALTA]\e[0m %s\n' "$*"; }

echo "== Sistema"
echo "  $(lsb_release -ds 2>/dev/null) | sesión: ${XDG_SESSION_TYPE:-?} | DISPLAY=${DISPLAY:-<vacío>}"

echo "== Docker"
if command -v docker >/dev/null; then ok "$(docker --version)"; else bad "docker no instalado"; fi
if docker compose version >/dev/null 2>&1; then ok "$(docker compose version)"; else bad "plugin docker compose (sudo apt install docker-compose-plugin)"; fi
if docker info >/dev/null 2>&1; then ok "Docker responde sin sudo"; else warn "docker info falla sin sudo: agrega tu usuario al grupo docker (sudo usermod -aG docker \$USER) y vuelve a iniciar sesión, o usa sudo"; fi

echo "== Pantalla"
if command -v xauth >/dev/null; then ok "xauth instalado"; else bad "xauth (sudo apt install xauth)"; fi
if [[ -n "${DISPLAY:-}" ]]; then ok "DISPLAY=${DISPLAY} (XWayland)"; else bad "DISPLAY vacío: abre una terminal dentro de la sesión gráfica"; fi
if command -v xhost >/dev/null; then ok "xhost instalado"; else warn "xhost no instalado (sudo apt install x11-xserver-utils); solo se usa como respaldo"; fi

echo "== GPU NVIDIA"
if nvidia-smi >/dev/null 2>&1; then
  ok "$(nvidia-smi --query-gpu=name,driver_version,memory.total --format=csv,noheader)"
  DRV="$(nvidia-smi --query-gpu=driver_version --format=csv,noheader | head -n1 | cut -d. -f1)"
  if [[ "${DRV:-0}" -ge 525 ]]; then ok "driver >= 525 (sirve para PyTorch con CUDA 12.1)"; else warn "driver ${DRV} < 525: actualiza el driver para usar CUDA 12.1"; fi
else
  warn "nvidia-smi no funciona: sin driver NVIDIA solo podrás usar YOLO en CPU"
fi
if command -v nvidia-ctk >/dev/null; then ok "nvidia-container-toolkit instalado"; else warn "nvidia-container-toolkit no instalado (ver README, sección GPU)"; fi
if docker info 2>/dev/null | grep -qi nvidia; then ok "Docker tiene runtime nvidia"; else warn "Docker sin runtime nvidia (sudo nvidia-ctk runtime configure --runtime=docker && sudo systemctl restart docker)"; fi

echo "== Dispositivos"
if ls /dev/video* >/dev/null 2>&1; then ok "cámaras: $(ls /dev/video* | tr '\n' ' ')"; else warn "no hay /dev/video* (webcam no detectada)"; fi
if ls /dev/ttyUSB* /dev/ttyACM* >/dev/null 2>&1; then ok "seriales: $(ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null | tr '\n' ' ')"; else warn "no hay /dev/ttyUSB* (normal si el Pioneer no está conectado)"; fi
if ls /dev/dri/renderD* >/dev/null 2>&1; then ok "/dev/dri disponible (OpenGL para RViz)"; else warn "no hay /dev/dri: RViz usará render por software"; fi
if dpkg -l brltty 2>/dev/null | grep -q '^ii'; then warn "brltty instalado: en Ubuntu 22.04+ a veces captura adaptadores USB-serial (CH340). Si /dev/ttyUSB0 aparece y desaparece: sudo apt remove brltty"; else ok "brltty no instalado"; fi
if id -nG | grep -qw dialout; then ok "usuario en grupo dialout"; else warn "usuario fuera de dialout (solo importa para usar el serial desde el host)"; fi
