#!/usr/bin/env bash
set -euo pipefail

# Virtual display for GUI apps (no XQuartz needed)
export DISPLAY="${DISPLAY:-:1}"
export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/tmp/runtime-root}"
mkdir -p "${XDG_RUNTIME_DIR}"
chmod 700 "${XDG_RUNTIME_DIR}"

# Prefer software rendering (Docker Desktop for Mac has no real GPU passthrough)
export QT_X11_NO_MITSHM=1
export QT_OPENGL=software
export LIBGL_ALWAYS_SOFTWARE=1
export QT_XCB_FORCE_SOFTWARE_OPENGL=1

VNC_PORT="${VNC_PORT:-5900}"
NOVNC_PORT="${NOVNC_PORT:-6080}"
GEOMETRY="${GEOMETRY:-1600x900}"
DEPTH="${DEPTH:-24}"

echo "[start-vnc] Starting Xvfb on ${DISPLAY} (${GEOMETRY}x${DEPTH})"
Xvfb "${DISPLAY}" -screen 0 "${GEOMETRY}x${DEPTH}" -nolisten tcp +extension GLX +render -noreset &

echo "[start-vnc] Starting window manager (fluxbox)"
fluxbox >/tmp/fluxbox.log 2>&1 &

echo "[start-vnc] Starting x11vnc on :${VNC_PORT}"
x11vnc -display "${DISPLAY}" -forever -shared -nopw -rfbport "${VNC_PORT}" >/tmp/x11vnc.log 2>&1 &

NOVNC_WEB_DIR="/usr/share/novnc"
if [[ ! -d "${NOVNC_WEB_DIR}" ]]; then
  # Some distros place it here
  NOVNC_WEB_DIR="/usr/share/novnc"
fi

echo "[start-vnc] Starting noVNC web on :${NOVNC_PORT} (serving ${NOVNC_WEB_DIR})"
websockify --web "${NOVNC_WEB_DIR}" "${NOVNC_PORT}" "localhost:${VNC_PORT}" >/tmp/novnc.log 2>&1 &

echo "[start-vnc] Ready. Open: http://localhost:${NOVNC_PORT}/vnc.html"
echo "[start-vnc] In this shell, run e.g.: ign gazebo -v 4 -r /usr/share/ignition/ignition-gazebo6/worlds/empty.sdf"

# Keep container alive both with and without TTY.
# - `docker run -it ... start-vnc.sh` -> interactive shell
# - `docker run -d  ... start-vnc.sh` -> stay alive so you can `docker exec`
if [[ -t 0 ]]; then
  exec bash -l
else
  tail -f /dev/null
fi

