#!/usr/bin/env bash
# Run an experiment script either inside an already-running Isaac Sim
# container, or via Docker Compose on a GPU host.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
EXP="${1:-}"

usage() {
  cat <<'EOF'
Usage: docker/run.sh <exp01|exp02|exp03|exp04|all|shell>

  exp01   Headless startup and GPU detection
  exp02   Ground plane + rigid cube, stepped simulation
  exp03   Simulated camera RGB capture
  exp04   Move the cube and capture frames
  all     Run exp01 through exp04 in order
  shell   Interactive bash in the Isaac Sim container
EOF
}

script_for() {
  case "$1" in
    exp01) echo "exp01_startup.py" ;;
    exp02) echo "exp02_simple_scene.py" ;;
    exp03) echo "exp03_camera.py" ;;
    exp04) echo "exp04_motion.py" ;;
    *) return 1 ;;
  esac
}

in_isaac_container() {
  [[ -x /isaac-sim/python.sh ]]
}

run_python() {
  local script_name="$1"
  local script_path="${ROOT}/src/${script_name}"
  if [[ ! -f "${script_path}" ]]; then
    echo "Missing script: ${script_path}" >&2
    exit 1
  fi

  if in_isaac_container; then
    echo "Running inside Isaac Sim container: ${script_path}"
    exec /isaac-sim/python.sh "${script_path}"
  fi

  if ! command -v docker >/dev/null 2>&1; then
    echo "Docker not found, and this is not an Isaac Sim container." >&2
    echo "Install Docker + NVIDIA Container Toolkit, or run from nvcr.io/nvidia/isaac-sim." >&2
    exit 1
  fi

  mkdir -p "${ROOT}/.cache/isaac-sim"/{main,computecache,logs,config,data,pkg}
  mkdir -p "${ROOT}/output"

  echo "Running via Docker Compose: ${script_name}"
  docker compose -f "${ROOT}/docker/docker-compose.yml" run --rm --workdir /isaac-sim isaac-sim \
    ./python.sh "/workspace/src/${script_name}"
}

run_shell() {
  if in_isaac_container; then
    exec bash
  fi
  mkdir -p "${ROOT}/.cache/isaac-sim"/{main,computecache,logs,config,data,pkg}
  mkdir -p "${ROOT}/output"
  docker compose -f "${ROOT}/docker/docker-compose.yml" run --rm isaac-sim
}

if [[ -z "${EXP}" ]]; then
  usage
  exit 1
fi

if [[ "${EXP}" == "-h" || "${EXP}" == "--help" ]]; then
  usage
  exit 0
fi

if [[ "${EXP}" == "shell" ]]; then
  run_shell
  exit 0
fi

if [[ "${EXP}" == "all" ]]; then
  if in_isaac_container; then
    for name in exp01 exp02 exp03 exp04; do
      /isaac-sim/python.sh "${ROOT}/src/$(script_for "${name}")"
    done
    exit 0
  fi
  for name in exp01 exp02 exp03 exp04; do
    run_python "$(script_for "${name}")"
  done
  exit 0
fi

if script_name="$(script_for "${EXP}")"; then
  run_python "${script_name}"
else
  usage
  exit 1
fi
