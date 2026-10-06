#!/data/data/com.termux/files/usr/bin/bash
# Start the GCS backend and the built frontend on the handheld as one local
# origin, then open http://127.0.0.1:8000 in the tablet browser.
#   bash scripts/termux/start_gcs.sh
set -euo pipefail

REPO="$(cd "$(dirname "$0")/../.." && pwd)"
DATA="${GCS_DATA_DIR:-$HOME/.gcs}"
PORT="${GCS_PORT:-8000}"
mkdir -p "$DATA"

export PYTHONPATH="$REPO/src"
export GCS_SETTINGS_PATH="$DATA/gcs_settings.json"
export GCS_LOG_DIR="$DATA/logs"
export GCS_FRONTEND_DIST="${GCS_FRONTEND_DIST:-$REPO/src/gcs/frontend/dist}"

# First start only: hardware defaults (no simulation, no dev UI, no automatic
# scan). The MAVLink device is set in the UI; which one the SIYI link needs is
# still open. Later UI changes are kept.
if [ ! -f "$GCS_SETTINGS_PATH" ]; then
    cat > "$GCS_SETTINGS_PATH" <<'JSON'
{
  "simulation": {"sim_mode": false, "dev_mode": false},
  "connection": {"auto_connect": false}
}
JSON
fi

# Keep the CPU running while the screen is off; release it however we exit.
termux-wake-lock || true
trap 'termux-wake-unlock || true' EXIT

# Loopback only: the API has no authentication.
python -m uvicorn gcs.backend.main:app --host 127.0.0.1 --port "$PORT"
