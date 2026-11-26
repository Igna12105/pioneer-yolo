#!/usr/bin/env bash
# Abre una terminal dentro del contenedor que esté corriendo (base o YOLO).
set -euo pipefail
NAME="$(docker ps --format '{{.Names}}' | grep -E '^pioneer_melodic(_yolo)?$' | head -n1 || true)"
if [[ -z "${NAME}" ]]; then
  echo "No hay contenedor del Pioneer corriendo. Inícialo con ./scripts/run_container.sh [--yolo] [--gpu]" >&2
  exit 1
fi
exec docker exec -it "${NAME}" bash
