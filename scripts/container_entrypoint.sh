#!/usr/bin/env bash
set -e

source /opt/ros/melodic/setup.bash

WS="${PIONEER_WS:-/root/pioneer_ws}"
if [[ -f "${WS}/src/rosaria/CMakeLists.txt" ]]; then
  cd "${WS}"
  if [[ ! -f devel/setup.bash ]]; then
    echo "[pioneer-entrypoint] First startup: building ROS workspace (catkin_make -j2)..."
    if ! catkin_make -j2; then
      echo "[pioneer-entrypoint] WARNING: catkin_make failed. The container will remain available for debugging." >&2
    fi
  fi
  if [[ -f devel/setup.bash ]]; then
    source "${WS}/devel/setup.bash"
  fi
else
  echo "[pioneer-entrypoint] No ROSARIA source found under ${WS}/src/rosaria. Run scripts/prepare_workspace.sh on the host." >&2
fi

exec "$@"
