#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ -z "${PIONEER_WS_HOST:-}" && -f "${ROOT_DIR}/.env" ]]; then
  set -a
  # shellcheck disable=SC1091
  source "${ROOT_DIR}/.env"
  set +a
fi
WS_DIR="${PIONEER_WS_HOST:-./pioneer_ws}"
if [[ "${WS_DIR}" != /* ]]; then WS_DIR="${ROOT_DIR}/${WS_DIR}"; fi
WITH_CLIENT=0
if [[ "${1:-}" == "--with-client" ]]; then WITH_CLIENT=1; fi

if [[ ! -d "${WS_DIR}/.git" ]]; then
  if [[ -e "${WS_DIR}" && -n "$(find "${WS_DIR}" -mindepth 1 -maxdepth 1 -print -quit 2>/dev/null)" ]]; then
    echo "ERROR: ${WS_DIR} exists and is not an empty Pioneer workspace." >&2
    echo "Move it aside or set PIONEER_WS_HOST to a new directory." >&2
    exit 1
  fi
  mkdir -p "$(dirname "${WS_DIR}")"
  git clone https://github.com/Uchile-Lab-Vision-Computacional/pioneer_ws.git "${WS_DIR}"
fi

mkdir -p "${WS_DIR}/src"
if [[ ! -d "${WS_DIR}/src/rosaria/.git" ]]; then
  git clone https://github.com/amor-ros-pkg/rosaria.git "${WS_DIR}/src/rosaria"
fi

if [[ "${WITH_CLIENT}" -eq 1 && ! -d "${WS_DIR}/src/rosaria_client/.git" ]]; then
  git clone https://github.com/pengtang/rosaria_client.git "${WS_DIR}/src/rosaria_client"
fi

printf '\nWorkspace ready: %s\n' "${WS_DIR}"
printf 'Mounted at /root/pioneer_ws inside the container.\n'
printf 'Optional rosaria_client: run this script with --with-client.\n'
