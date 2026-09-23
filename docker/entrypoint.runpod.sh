#!/usr/bin/env bash
# Keep a RunPod container alive and (optionally) start SSH using PUBLIC_KEY.
set -euo pipefail

# A Kit abort can write a multi-GB core file and OOM/kill the RunPod container
# (which drops SSH). Disable core dumps in this image.
ulimit -c 0 2>/dev/null || true

if [[ -n "${PUBLIC_KEY:-}" ]]; then
  mkdir -p /root/.ssh
  chmod 700 /root/.ssh
  touch /root/.ssh/authorized_keys
  chmod 600 /root/.ssh/authorized_keys
  if ! grep -qxF "${PUBLIC_KEY}" /root/.ssh/authorized_keys; then
    echo "${PUBLIC_KEY}" >> /root/.ssh/authorized_keys
  fi
fi

if [[ -f /etc/ssh/sshd_config ]]; then
  sed -i 's/^#\?PermitRootLogin.*/PermitRootLogin yes/' /etc/ssh/sshd_config
  sed -i 's/^#\?PasswordAuthentication.*/PasswordAuthentication no/' /etc/ssh/sshd_config
fi

if [[ -x /usr/sbin/sshd ]]; then
  /usr/sbin/sshd
fi

# Optional NVIDIA Xorg + noVNC desktop for EXP-06. Does not start Isaac Sim.
if [[ "${ISAAC_GUI:-}" == "1" && -x /usr/local/bin/isaac-gui.sh ]]; then
  /usr/local/bin/isaac-gui.sh start || true
fi

if [[ "$#" -gt 0 ]]; then
  exec "$@"
fi

exec sleep infinity
