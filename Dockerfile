# ROS Melodic (Ubuntu 18.04/Bionic) + Pioneer P3-DX + ROSARIA + AriaCoda
# Imagen oficial histórica con RViz/rqt y la pila gráfica de ROS.
FROM osrf/ros:melodic-desktop-full

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    ROS_DISTRO=melodic \
    PIONEER_WS=/root/pioneer_ws

ARG ARIACODA_REPO=https://github.com/reedhedges/AriaCoda.git
# Commit fijo (master al 2026-08-20) para que el build sea reproducible.
# Usa C++14 por defecto, compatible con g++ 7 de Bionic.
ARG ARIACODA_REF=b0d02e05888f9317eb2cedbac2b6f14deabdd6ee
ARG BUILD_JOBS=4

# --- Repositorios apt -------------------------------------------------------
# 1) La llave del repo snapshots.ros.org ha expirado varias veces (2023, 2025).
#    Se vuelve a descargar desde el keyserver para tomar la fecha de expiración
#    vigente. Si el keyserver no responde se sigue con la llave de la imagen.
# 2) Bionic sigue en archive.ubuntu.com (está en ESM hasta 2028). Solo si eso
#    falla se cambia a old-releases.ubuntu.com.
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 \
      --recv-keys 4B63CF8FDE49746E98FA01DDAD19BAB3CBF125EA \
    || echo "WARN: no se pudo refrescar la llave de snapshots.ros.org; se usa la de la imagen"

RUN if ! apt-get update; then \
      echo "WARN: apt-get update falló; probando old-releases.ubuntu.com"; \
      sed -i -e 's|archive.ubuntu.com/ubuntu|old-releases.ubuntu.com/ubuntu|g' \
             -e 's|security.ubuntu.com/ubuntu|old-releases.ubuntu.com/ubuntu|g' \
             /etc/apt/sources.list && apt-get update; \
    fi \
    && apt-get install -y --no-install-recommends \
      build-essential cmake git make g++ pkg-config \
      libboost-all-dev \
      python-rosdep python-rosinstall python-vcstools \
      python3-pip python3-venv python3-dev \
      udev usbutils x11-apps mesa-utils v4l-utils \
      libgl1-mesa-glx libgl1-mesa-dri libglib2.0-0 \
      ros-melodic-rqt ros-melodic-rqt-image-view ros-melodic-rqt-graph \
      ros-melodic-message-generation ros-melodic-message-runtime \
      ros-melodic-roscpp ros-melodic-nav-msgs ros-melodic-geometry-msgs \
      ros-melodic-sensor-msgs ros-melodic-std-msgs ros-melodic-std-srvs \
      ros-melodic-tf ros-melodic-dynamic-reconfigure \
      ros-melodic-robot-state-publisher ros-melodic-joint-state-publisher \
      ros-melodic-teleop-twist-keyboard \
      ros-melodic-usb-cam ros-melodic-cv-bridge ros-melodic-image-transport \
    && rm -rf /var/lib/apt/lists/*

# --- AriaCoda ---------------------------------------------------------------
# Se instala en /usr/local (include/Aria/Aria.h y lib/libAria.so), que es donde
# lo busca el CMakeLists.txt de rosaria. Se usa install-default + install-utils
# en vez de "install" porque este último también genera la documentación con
# doxygen (no instalado y no hace falta).
RUN git clone "${ARIACODA_REPO}" /tmp/AriaCoda \
    && git -C /tmp/AriaCoda checkout "${ARIACODA_REF}" \
    && make -C /tmp/AriaCoda -j"${BUILD_JOBS}" \
    && make -C /tmp/AriaCoda install-default install-utils \
    && ldconfig \
    && test -f /usr/local/include/Aria/Aria.h \
    && test -f /usr/local/lib/libAria.so \
    && rm -rf /tmp/AriaCoda

# No se toca el Python del sistema que usa ROS Melodic. El workspace montado se
# carga en las shells interactivas una vez que catkin_make crea devel/setup.bash.
COPY scripts/container_entrypoint.sh /container_entrypoint_pioneer.sh
RUN chmod +x /container_entrypoint_pioneer.sh \
    && printf '\n# Pioneer workspace overlay\nsource /opt/ros/melodic/setup.bash\nif [ -f /root/pioneer_ws/devel/setup.bash ]; then\n  source /root/pioneer_ws/devel/setup.bash\nfi\n' >> /root/.bashrc

WORKDIR /root/pioneer_ws
ENTRYPOINT ["/container_entrypoint_pioneer.sh"]
CMD ["bash", "-lc", "sleep infinity"]
