#!/usr/bin/env bash
# EXP-06: Isaac Sim GUI on an NVIDIA Xorg dummy screen, viewed from a Mac over TCP.
#
# Proven 2026-08-30 on RunPod RTX 4090:
#   Isaac (DISPLAY=:1) → Xorg + NVIDIA dummy → x11vnc :5900 → noVNC :8080 → SSH -L
#
# Do not use Xvfb (Kit: backbuffers are not initialized).
# Do not use NVIDIA WebRTC on RunPod (no inbound UDP 47998).
# Do not apt-install xserver-xorg-video-nvidia-* (version mismatch).
# If the host did not inject nvidia_drv.so, extract it from the matching NVIDIA .run.
set -euo pipefail

DISPLAY_NUM="${ISAAC_GUI_DISPLAY:-1}"
export DISPLAY=":${DISPLAY_NUM}"
RES="${ISAAC_GUI_RES:-1920x1080}"
NOVNC_PORT="${ISAAC_GUI_PORT:-8080}"
VNC_PORT="${ISAAC_VNC_PORT:-5900}"
STATE_DIR="${ISAAC_GUI_STATE:-/tmp/isaac-gui}"
NOVNC_WEB="${NOVNC_WEB:-/usr/share/novnc}"

GUI_APT_PACKAGES=(
  xorg
  xserver-xorg-core
  xinit
  x11-xserver-utils
  xauth
  mesa-utils
  x11vnc
  novnc
  websockify
  openbox
  xterm
  x11-apps
  x11-utils
  dbus-x11
  procps
  openssl
  wmctrl
  curl
)

isaac_bin() {
  if [[ -x /isaac-sim/isaac-sim.sh ]]; then
    echo /isaac-sim/isaac-sim.sh
  elif [[ -x /isaac-sim/runapp.sh ]]; then
    echo /isaac-sim/runapp.sh
  else
    echo ""
  fi
}

usage() {
  cat <<EOF
Usage: docker/gui.sh <start|isaac|smoke|status|stop|probe>

  start   NVIDIA Xorg dummy + noVNC (does not start Isaac Sim)
  isaac   Launch the Isaac Sim GUI on that display (use after start)
  smoke   Open xclock so you can test the Mac browser before Kit
  status  Show DISPLAY, ports, and related processes
  probe   Check GPU devices, nvidia_drv.so, and GLX renderer
  stop    Stop noVNC / VNC / window manager / Xorg (not Kit unless --isaac)

Environment:
  ISAAC_GUI_RES       Virtual screen hint, default ${RES}
  ISAAC_GUI_PORT      noVNC HTTP port, default ${NOVNC_PORT}
  VNC_PASSWORD        VNC password (generated if unset; printed on start)
  ISAAC_GUI=1         From the image entrypoint: run "start" when the pod boots

On the Mac, use TCP SSH (Connect → SSH over exposed TCP), not ssh.runpod.io:

  ssh -L ${NOVNC_PORT}:127.0.0.1:${NOVNC_PORT} root@<PUBLIC_IP> -p <TCP_PORT> -i ~/.ssh/id_ed25519
  open http://127.0.0.1:${NOVNC_PORT}/vnc.html?autoconnect=1

Do not run docker/run.sh exp0N while Kit GUI is using the GPU.
Quit Isaac Sim with File > Exit in the streamed window, then: docker/gui.sh stop
EOF
}

need_cmd() {
  local name="$1"
  if ! command -v "${name}" >/dev/null 2>&1; then
    echo "Missing ${name}. Run: docker/gui.sh start (installs packages) or rebuild the image." >&2
    exit 1
  fi
}

websockify_cmd() {
  if command -v websockify >/dev/null 2>&1; then
    echo websockify
    return
  fi
  if python3 -c "import websockify" >/dev/null 2>&1; then
    echo python3 -m websockify
    return
  fi
  echo "Missing websockify. Run docker/gui.sh start or rebuild the image." >&2
  exit 1
}

ensure_packages() {
  local missing=0
  local cmd
  for cmd in Xorg x11vnc openbox xdpyinfo glxinfo; do
    if ! command -v "${cmd}" >/dev/null 2>&1; then
      missing=1
      break
    fi
  done
  if [[ "${missing}" -eq 0 ]]; then
    return 0
  fi
  if ! command -v apt-get >/dev/null 2>&1; then
    echo "GUI packages missing and apt-get is not available." >&2
    exit 1
  fi
  echo "[isaac-gui] Installing Xorg / noVNC packages (one-time on this pod)..."
  export DEBIAN_FRONTEND=noninteractive
  apt-get update
  apt-get install -y --no-install-recommends "${GUI_APT_PACKAGES[@]}"
}

write_password() {
  mkdir -p "${STATE_DIR}"
  chmod 700 "${STATE_DIR}"
  local password="${VNC_PASSWORD:-}"
  if [[ -z "${password}" && -f "${STATE_DIR}/password.txt" ]]; then
    password="$(cat "${STATE_DIR}/password.txt")"
  fi
  if [[ -z "${password}" ]]; then
    password="$(openssl rand -base64 12 | tr -d '/+=' | head -c 12)"
  fi
  printf '%s' "${password}" > "${STATE_DIR}/password.txt"
  chmod 600 "${STATE_DIR}/password.txt"
  x11vnc -storepasswd "${password}" "${STATE_DIR}/vnc.pass" >/dev/null
  echo "${password}"
}

nvidia_bus_id() {
  python3 - <<'PY'
import subprocess
import sys
try:
    raw = subprocess.check_output(
        ["nvidia-smi", "--query-gpu=pci.bus_id", "--format=csv,noheader"],
        text=True,
    ).strip().splitlines()[0]
except (subprocess.CalledProcessError, FileNotFoundError, IndexError) as exc:
    print(f"nvidia-smi pci.bus_id failed: {exc}", file=sys.stderr)
    sys.exit(1)
parts = raw.replace(".", ":").split(":")
bus = int(parts[-3], 16)
dev = int(parts[-2], 16)
func = int(parts[-1], 16)
print(f"PCI:{bus}:{dev}:{func}")
PY
}

write_xorg_conf() {
  local busid="$1"
  local conf="${STATE_DIR}/xorg.conf"
  cat > "${conf}" <<EOF
Section "ServerLayout"
    Identifier "layout"
    Screen 0 "screen0"
EndSection
Section "Files"
    ModulePath "/usr/lib/xorg/modules"
EndSection
Section "Device"
    Identifier "nvidia"
    Driver "nvidia"
    BusID "${busid}"
    Option "AllowEmptyInitialConfiguration" "True"
    Option "ConnectedMonitor" "DFP-0"
    Option "UseDisplayDevice" "DFP-0"
    Option "MetaModes" "${RES} +0+0"
EndSection
Section "Screen"
    Identifier "screen0"
    Device "nvidia"
    DefaultDepth 24
    Option "AllowEmptyInitialConfiguration" "True"
    SubSection "Display"
        Depth 24
        Virtual ${RES/x/ }
        Modes "${RES}"
    EndSubSection
EndSection
EOF
  echo "${conf}"
}

find_nvidia_drv() {
  local path
  for path in \
    /usr/lib/xorg/modules/drivers/nvidia_drv.so \
    /usr/lib/x86_64-linux-gnu/nvidia/xorg/nvidia_drv.so; do
    if [[ -f "${path}" ]]; then
      echo "${path}"
      return 0
    fi
  done
  find /usr /opt -name 'nvidia_drv.so' 2>/dev/null | head -1 || true
}

nvidia_driver_version() {
  nvidia-smi --query-gpu=driver_version --format=csv,noheader 2>/dev/null \
    | head -1 | tr -d '[:space:]'
}

# Host often injects CUDA but not nvidia_drv.so. Extract the matching Xorg
# modules from NVIDIA's .run (do not install the kernel driver). Cache under
# /var/cache so a pod stop/start does not re-download; a new pod will.
ensure_nvidia_xorg_modules() {
  mkdir -p /usr/lib/xorg/modules/drivers /usr/lib/xorg/modules/extensions
  if [[ -f /usr/lib/xorg/modules/drivers/nvidia_drv.so ]]; then
    return 0
  fi
  if ! command -v nvidia-smi >/dev/null 2>&1; then
    echo "nvidia-smi missing; cannot match an Xorg driver." >&2
    exit 1
  fi
  local ver
  ver="$(nvidia_driver_version)"
  if [[ ! "${ver}" =~ ^[0-9]+\.[0-9.]+$ ]]; then
    echo "Could not read NVIDIA driver version from nvidia-smi." >&2
    exit 1
  fi
  local cache="${ISAAC_GUI_NVIDIA_CACHE:-/var/cache/isaac-gui/nvidia-xorg/${ver}}"
  local runfile="${cache}/NVIDIA-Linux-x86_64-${ver}.run"
  mkdir -p "${cache}"
  if [[ ! -f "${cache}/nvidia_drv.so" ]]; then
    if [[ ! -f "${runfile}" ]]; then
      echo "[isaac-gui] Host did not inject nvidia_drv.so. Downloading NVIDIA ${ver} Xorg modules..."
      local url="https://us.download.nvidia.com/XFree86/Linux-x86_64/${ver}/NVIDIA-Linux-x86_64-${ver}.run"
      if ! curl -fL --retry 3 -o "${runfile}.partial" "${url}"; then
        rm -f "${runfile}.partial"
        echo "Download failed: ${url}" >&2
        echo "Xorg GUI needs nvidia_drv.so matching the host driver ${ver}." >&2
        exit 1
      fi
      mv "${runfile}.partial" "${runfile}"
    fi
    chmod +x "${runfile}"
    echo "[isaac-gui] Extracting ${runfile} (not installing)..."
    (
      cd "${cache}"
      sh "${runfile}" --extract-only
    )
    local extracted
    extracted="$(find "${cache}/NVIDIA-Linux-x86_64-${ver}" -name 'nvidia_drv.so' | head -1)"
    if [[ -z "${extracted}" ]]; then
      echo "nvidia_drv.so missing after extract of ${ver}." >&2
      exit 1
    fi
    cp "${extracted}" "${cache}/nvidia_drv.so"
    local glx
    glx="$(find "${cache}/NVIDIA-Linux-x86_64-${ver}" -name 'libglxserver_nvidia.so*' | head -1)"
    if [[ -n "${glx}" ]]; then
      cp "${glx}" "${cache}/libglxserver_nvidia.so"
    fi
  fi
  cp "${cache}/nvidia_drv.so" /usr/lib/xorg/modules/drivers/nvidia_drv.so
  if [[ -f "${cache}/libglxserver_nvidia.so" ]]; then
    cp "${cache}/libglxserver_nvidia.so" /usr/lib/xorg/modules/extensions/libglxserver_nvidia.so
  fi
  chmod 755 /usr/lib/xorg/modules/drivers/nvidia_drv.so
  echo "[isaac-gui] Installed nvidia_drv.so for driver ${ver} (cached in ${cache})"
}

try_set_framebuffer() {
  if ! command -v xrandr >/dev/null 2>&1; then
    return 0
  fi
  local current
  current="$(DISPLAY=":${DISPLAY_NUM}" xrandr 2>/dev/null | awk '/current/{print $8"x"$10; exit}' | tr -d ',')"
  if [[ "${current}" == "8x8" ]]; then
    echo "[isaac-gui] Dummy screen is 8x8; setting ${RES}"
    DISPLAY=":${DISPLAY_NUM}" xrandr --fb "${RES}" || true
  fi
}

is_running() {
  local pid_file="$1"
  [[ -f "${pid_file}" ]] && kill -0 "$(cat "${pid_file}")" 2>/dev/null
}

start_bg() {
  local name="$1"
  local pid_file="${STATE_DIR}/${name}.pid"
  shift
  if is_running "${pid_file}"; then
    echo "${name} already running (pid $(cat "${pid_file}"))"
    return 0
  fi
  mkdir -p "${STATE_DIR}"
  nohup "$@" >"${STATE_DIR}/${name}.log" 2>&1 &
  echo $! > "${pid_file}"
  echo "started ${name} pid $!"
}

stop_leftover_xvfb() {
  if pgrep -f "Xvfb :${DISPLAY_NUM}" >/dev/null 2>&1; then
    echo "[isaac-gui] Stopping leftover Xvfb on :${DISPLAY_NUM} (cannot host Isaac RTX)."
    pkill -f "Xvfb :${DISPLAY_NUM}" || true
    sleep 0.3
  fi
}

cmd_probe() {
  echo "DISPLAY=${DISPLAY} RES=${RES} noVNC_port=${NOVNC_PORT}"
  echo "NVIDIA_DRIVER_CAPABILITIES=${NVIDIA_DRIVER_CAPABILITIES:-unset}"
  if command -v nvidia-smi >/dev/null 2>&1; then
    nvidia-smi
    echo "Xorg BusID: $(nvidia_bus_id || echo failed)"
  else
    echo "nvidia-smi: missing"
  fi
  echo "--- devices ---"
  ls -l /dev/nvidia* /dev/dri 2>/dev/null || echo "no /dev/nvidia* or /dev/dri"
  echo "nvidia_drv.so: $(find_nvidia_drv || echo missing)"
  if xdpyinfo -display ":${DISPLAY_NUM}" >/dev/null 2>&1; then
    echo "--- glxinfo -B ---"
    DISPLAY=":${DISPLAY_NUM}" glxinfo -B || true
  else
    echo "Display ${DISPLAY} is down (start it with: docker/gui.sh start)"
  fi
}

cmd_start() {
  ensure_packages
  need_cmd Xorg
  need_cmd x11vnc
  need_cmd openbox
  local ws
  ws="$(websockify_cmd)"
  if [[ ! -f "${NOVNC_WEB}/vnc.html" ]]; then
    for candidate in /usr/share/novnc /usr/share/novnc/app; do
      if [[ -f "${candidate}/vnc.html" ]]; then
        NOVNC_WEB="${candidate}"
        break
      fi
    done
  fi
  if [[ ! -f "${NOVNC_WEB}/vnc.html" ]]; then
    echo "noVNC vnc.html not found (looked in ${NOVNC_WEB})" >&2
    exit 1
  fi

  ensure_nvidia_xorg_modules
  local drv
  drv="$(find_nvidia_drv)"
  if [[ -z "${drv}" ]]; then
    echo "nvidia_drv.so still missing after extract." >&2
    exit 1
  fi
  if [[ ! -e /dev/nvidiactl ]]; then
    echo "NVIDIA device nodes missing. Check RunPod GPU passthrough." >&2
    exit 1
  fi

  mkdir -p "${STATE_DIR}"
  stop_leftover_xvfb

  local password busid conf
  password="$(write_password)"
  busid="$(nvidia_bus_id)"
  conf="$(write_xorg_conf "${busid}")"
  echo "[isaac-gui] NVIDIA BusID ${busid}  driver ${drv}"

  if [[ -z "${DBUS_SESSION_BUS_ADDRESS:-}" ]] && command -v dbus-launch >/dev/null 2>&1; then
    # shellcheck disable=SC2046
    eval "$(dbus-launch --sh-syntax)"
    printf '%s\n' "${DBUS_SESSION_BUS_ADDRESS}" > "${STATE_DIR}/dbus.address"
  fi

  if ! xdpyinfo -display ":${DISPLAY_NUM}" >/dev/null 2>&1; then
    start_bg xorg Xorg ":${DISPLAY_NUM}" -config "${conf}" \
      -noreset +extension GLX -nolisten tcp
  else
    echo "X display ${DISPLAY} already up"
  fi

  local i
  for i in $(seq 1 100); do
    if xdpyinfo -display ":${DISPLAY_NUM}" >/dev/null 2>&1; then
      break
    fi
    sleep 0.2
  done
  if ! xdpyinfo -display ":${DISPLAY_NUM}" >/dev/null 2>&1; then
    echo "Xorg did not become ready. See ${STATE_DIR}/xorg.log and /var/log/Xorg.${DISPLAY_NUM}.log" >&2
    tail -n 40 "${STATE_DIR}/xorg.log" 2>/dev/null || true
    exit 1
  fi

  local renderer
  renderer="$(DISPLAY=":${DISPLAY_NUM}" glxinfo -B 2>/dev/null | awk -F': ' '/OpenGL renderer string/{print $2}')"
  echo "[isaac-gui] GLX renderer: ${renderer:-unknown}"
  if [[ "${renderer}" == *llvmpipe* || "${renderer}" == *softpipe* ]]; then
    echo "GLX is software rasterization, not NVIDIA. Isaac will not present." >&2
    exit 1
  fi
  try_set_framebuffer

  start_bg openbox openbox --sm-disable
  start_bg x11vnc x11vnc -display ":${DISPLAY_NUM}" -rfbport "${VNC_PORT}" \
    -rfbauth "${STATE_DIR}/vnc.pass" -forever -shared -localhost \
    -noxdamage -repeat
  # shellcheck disable=SC2086
  start_bg novnc ${ws} --web "${NOVNC_WEB}" \
    "0.0.0.0:${NOVNC_PORT}" "127.0.0.1:${VNC_PORT}"

  cat <<EOF
[isaac-gui] DISPLAY=${DISPLAY}  ${RES}
[isaac-gui] noVNC http://127.0.0.1:${NOVNC_PORT}/vnc.html?autoconnect=1
[isaac-gui] VNC password: ${password}
[isaac-gui] Mac: ssh -L ${NOVNC_PORT}:127.0.0.1:${NOVNC_PORT} root@<PUBLIC_IP> -p <TCP_SSH_PORT>
[isaac-gui] Next: docker/gui.sh smoke   then   docker/gui.sh isaac
EOF
}

cmd_smoke() {
  if ! xdpyinfo -display ":${DISPLAY_NUM}" >/dev/null 2>&1; then
    echo "Display ${DISPLAY} is down. Run: docker/gui.sh start" >&2
    exit 1
  fi
  if command -v xclock >/dev/null 2>&1; then
    start_bg xclock xclock
    echo "xclock is on the virtual desktop. Connect with noVNC to confirm the Mac viewer."
  elif command -v xterm >/dev/null 2>&1; then
    start_bg xterm xterm
    echo "xterm is on the virtual desktop. Connect with noVNC to confirm the Mac viewer."
  else
    echo "Install x11-apps (xclock) for a smoke test." >&2
    exit 1
  fi
}

cmd_isaac() {
  local bin
  bin="$(isaac_bin)"
  if [[ -z "${bin}" ]]; then
    echo "Isaac Sim GUI launcher not found under /isaac-sim." >&2
    exit 1
  fi
  if ! xdpyinfo -display ":${DISPLAY_NUM}" >/dev/null 2>&1; then
    echo "Display ${DISPLAY} is down. Run: docker/gui.sh start" >&2
    exit 1
  fi
  if pgrep -f '/isaac-sim/kit/kit' >/dev/null 2>&1; then
    echo "A Kit process is already running. Stop it before starting the GUI:" >&2
    echo "  nvidia-smi" >&2
    echo "  kill \$(pgrep -f '/isaac-sim/kit/kit')" >&2
    exit 1
  fi

  export ACCEPT_EULA="${ACCEPT_EULA:-Y}"
  export PRIVACY_CONSENT="${PRIVACY_CONSENT:-Y}"
  export OMNI_KIT_ALLOW_ROOT="${OMNI_KIT_ALLOW_ROOT:-1}"
  export __GLX_VENDOR_LIBRARY_NAME="${__GLX_VENDOR_LIBRARY_NAME:-nvidia}"
  if [[ -f /usr/share/vulkan/icd.d/nvidia_icd.json ]]; then
    export VK_ICD_FILENAMES="${VK_ICD_FILENAMES:-/usr/share/vulkan/icd.d/nvidia_icd.json}"
  fi

  echo "Launching ${bin} on DISPLAY=${DISPLAY} (first start can take several minutes)."
  echo "Log: ${STATE_DIR}/isaac.log"
  mkdir -p "${STATE_DIR}"
  if is_running "${STATE_DIR}/isaac.pid"; then
    echo "isaac already running (pid $(cat "${STATE_DIR}/isaac.pid"))"
    return 0
  fi
  (
    cd /isaac-sim
    nohup "${bin}" --allow-root \
      --/app/window/width="${RES%x*}" \
      --/app/window/height="${RES#*x}" \
      >"${STATE_DIR}/isaac.log" 2>&1 &
    echo $! > "${STATE_DIR}/isaac.pid"
  )
  echo "started isaac pid $(cat "${STATE_DIR}/isaac.pid")"
  echo "When the viewport appears, continue EXP-07 / EXP-08. Quit with File > Exit."
}

cmd_status() {
  echo "DISPLAY=${DISPLAY} RES=${RES} noVNC_port=${NOVNC_PORT}"
  if [[ -f "${STATE_DIR}/password.txt" ]]; then
    echo "VNC password: $(cat "${STATE_DIR}/password.txt")"
  fi
  local name
  for name in xorg openbox x11vnc novnc isaac xclock xterm xvfb; do
    if is_running "${STATE_DIR}/${name}.pid"; then
      echo "${name}: running pid $(cat "${STATE_DIR}/${name}.pid")"
    else
      echo "${name}: stopped"
    fi
  done
  if command -v nvidia-smi >/dev/null 2>&1; then
    nvidia-smi --query-compute-apps=pid,process_name,used_memory --format=csv || true
  fi
}

stop_one() {
  local name="$1"
  local pid_file="${STATE_DIR}/${name}.pid"
  if is_running "${pid_file}"; then
    kill "$(cat "${pid_file}")" 2>/dev/null || true
    sleep 0.2
    if is_running "${pid_file}"; then
      kill -9 "$(cat "${pid_file}")" 2>/dev/null || true
    fi
  fi
  rm -f "${pid_file}"
}

cmd_stop() {
  local stop_isaac=0
  if [[ "${1:-}" == "--isaac" ]]; then
    stop_isaac=1
  fi
  if [[ "${stop_isaac}" == 1 ]]; then
    stop_one isaac
    pkill -f '/isaac-sim/kit/kit' 2>/dev/null || true
  elif is_running "${STATE_DIR}/isaac.pid" || pgrep -f '/isaac-sim/kit/kit' >/dev/null 2>&1; then
    echo "Isaac Sim is still running. File > Exit in the GUI, or: docker/gui.sh stop --isaac"
  fi
  stop_one xclock
  stop_one xterm
  stop_one novnc
  stop_one x11vnc
  stop_one openbox
  stop_one xorg
  stop_one xvfb
  stop_leftover_xvfb
  echo "GUI desktop stopped."
}

CMD="${1:-}"
case "${CMD}" in
  start) cmd_start ;;
  isaac) cmd_isaac ;;
  smoke) cmd_smoke ;;
  status) cmd_status ;;
  probe) cmd_probe ;;
  stop) cmd_stop "${2:-}" ;;
  -h|--help|"") usage ;;
  *)
    usage
    exit 1
    ;;
esac
