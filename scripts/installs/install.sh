#!/usr/bin/env bash
# nano install.sh
# chmod +x install.sh
# ./install.sh
#
# Works on Raspberry Pi OS / generic Ubuntu and on NVIDIA Jetson (JetPack 6.x).
# On Jetson the venv is created with --system-site-packages so the CUDA-enabled
# system python3-opencv is reused instead of the CPU-only pip wheel.

set -euo pipefail

REPO_BRANCH="dev"

# ── Detect platform ─────────────────────────────────────────────────────────
IS_JETSON=0
if [ -f /etc/nv_tegra_release ] || [ -d /proc/device-tree/nvidia,tegra-cpufreq ] \
  || grep -qi tegra /proc/device-tree/compatible 2>/dev/null; then
  IS_JETSON=1
  echo "› Detected NVIDIA Jetson platform."
fi

RUN_USER="${SUDO_USER:-$USER}"
USER_HOME="$(getent passwd "$RUN_USER" | cut -d: -f6)"
REPO_DIR="$USER_HOME/navpy"
VENV_DIR="$REPO_DIR/.venv"

echo "Updating system packages…"
sudo apt-get update

echo "Installing prerequisites…"
sudo apt-get install -y git python3 python3-venv python3-pip screen

if [ "$IS_JETSON" -eq 1 ]; then
  echo "› Installing Jetson system packages (CUDA-enabled OpenCV, Tk)…"
  sudo apt-get install -y python3-opencv python3-tk
fi

# ── Clone or refresh repo ────────────────────────────────────────────────────
if [ ! -d "$REPO_DIR" ]; then
  cat <<'EOF'
Cloning the private repository may prompt for GitHub credentials.
Use your GitHub username and a personal access token as the password.
The clean repository URL keeps the token out of .git/config.
EOF
  git clone --branch "$REPO_BRANCH" --single-branch \
    "https://github.com/gartfeo/navpy.git" "$REPO_DIR"
elif [ -d "$REPO_DIR/.git" ]; then
  echo "› navpy already cloned — switching to $REPO_BRANCH and pulling latest…"
  git -C "$REPO_DIR" fetch origin "$REPO_BRANCH"
  if git -C "$REPO_DIR" show-ref --verify --quiet "refs/heads/$REPO_BRANCH"; then
    git -C "$REPO_DIR" checkout "$REPO_BRANCH"
  else
    git -C "$REPO_DIR" checkout --track -b "$REPO_BRANCH" "origin/$REPO_BRANCH"
  fi
  git -C "$REPO_DIR" pull --ff-only origin "$REPO_BRANCH"
else
  echo "ERROR: $REPO_DIR exists but is not a git checkout. Remove it and re-run." >&2
  exit 1
fi

if [ ! -f "$REPO_DIR/pyproject.toml" ]; then
  echo "ERROR: $REPO_DIR/pyproject.toml missing — checkout looks corrupt." >&2
  exit 1
fi

cd "$REPO_DIR"

# ── Create venv if needed ───────────────────────────────────────────────────
if [ ! -d "$VENV_DIR" ]; then
  echo "› Creating virtualenv…"
  if [ "$IS_JETSON" -eq 1 ]; then
    # Inherit system site-packages so the CUDA cv2 / TensorRT bindings are visible.
    python3 -m venv --system-site-packages "$VENV_DIR"
  else
    python3 -m venv "$VENV_DIR"
  fi
fi

# ── Activate & install ──────────────────────────────────────────────────────
# shellcheck disable=SC1091
source "$VENV_DIR/bin/activate"
pip install --upgrade pip

if [ "$IS_JETSON" -eq 1 ]; then
  # Install the package without deps so pip does NOT pull opencv-python-headless
  # (which would shadow the CUDA system cv2). Then install the remaining deps
  # explicitly, omitting opencv.
  pip install --no-deps -e .
  pip install \
    "pymavlink @ git+https://github.com/gartfeo/mavlink.git@ArduPilot-4.6/navlink#egg=pymavlink&subdirectory=pymavlink" \
    pyserial numpy geopy pymap3d pyzmq requests pytest
else
  pip install -e .
  pip install pytest
fi

echo "› Running tests…"
pytest tests

# ── systemd service ─────────────────────────────────────────────────────────
sudo tee /etc/default/navpy >/dev/null <<'ENV'
# Serial example:
NAVPY_ARGS="-c /dev/ttyACM0 -b 115200 -nt mav -ss 158 -lsd Vehicle"

# IP example:
# NAVPY_ARGS="-c 192.168.13.1:15001 -nt mav -ss 158 -lsd Vehicle"

ENV

sudo tee /etc/systemd/system/navpy.service >/dev/null <<UNIT
[Unit]
Description=NavPy autopilot client (env-driven args)
Wants=network-online.target
After=network-online.target

[Service]
Type=simple
User=${RUN_USER}
WorkingDirectory=${REPO_DIR}
EnvironmentFile=/etc/default/navpy
ExecStart=/bin/bash -lc 'source /etc/default/navpy; exec ${VENV_DIR}/bin/python -m navpy.main \${NAVPY_ARGS}'
Restart=always
RestartSec=5s
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
UNIT

sudo systemctl daemon-reload
sudo systemctl enable --now navpy


# ── Persist navpy env & auto-update into ~/.bashrc ───────────────────────────
if ! grep -q "### navpy auto-update ###" "$USER_HOME/.bashrc"; then
  cat <<EOF >> "$USER_HOME/.bashrc"
### navpy auto-update ###
export PYTHONPATH="${REPO_DIR}/src:\$PYTHONPATH"
if [ -d "${VENV_DIR}" ]; then
  # shellcheck disable=SC1091
  source "${VENV_DIR}/bin/activate"
fi

if [ -d "${REPO_DIR}" ]; then
  cd "${REPO_DIR}" || return
  git fetch origin dev >/dev/null 2>&1
  LOCAL=\$(git rev-parse @)
  REMOTE=\$(git rev-parse @{u})
  BASE=\$(git merge-base @ @{u})
  if [ "\$LOCAL" = "\$BASE" ] && [ "\$REMOTE" != "\$BASE" ]; then
    echo "New navpy commits — pulling & testing…"
    git pull --no-edit >/dev/null
    pytest tests
  fi
  cd - >/dev/null
fi

systemctl status navpy.service

### end navpy auto-update ###
EOF
fi

echo
echo "Setup complete! Run 'source ~/.bashrc' or open a new shell to apply."

