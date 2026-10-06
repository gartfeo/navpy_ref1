#!/data/data/com.termux/files/usr/bin/bash
# One-time setup of the GCS backend on an Android handheld (SIYI UniRC 10 Pro)
# under Termux. Run from the repository copied onto the device:
#   bash scripts/termux/setup_gcs.sh
set -euo pipefail

REPO="$(cd "$(dirname "$0")/../.." && pwd)"

# Prebuilt Termux packages. opencv-python lives in the x11 repository; the
# backend imports cv2 through navpy's vision package.
pkg update -y
pkg install -y python python-pip python-numpy python-lxml git \
    rust binutils clang cmake ninja libzmq x11-repo
pkg install -y opencv-python

# pydantic-core and orjson (and the maturin tool that builds them) compile with
# rust. maturin tags its wheels with ANDROID_API_LEVEL; take the level Termux's
# python reports (Termux fixes it at build time, and it can differ from the
# device's SDK) so pip accepts those wheels. pyzmq compiles against Termux's
# libzmq instead of bundling its own. Expect this step to take long.
API_LEVEL="$(python -c 'import platform; print(platform.android_ver().api_level)' || true)"
if [ -z "$API_LEVEL" ] || [ "$API_LEVEL" = "0" ]; then
    API_LEVEL="$(getprop ro.build.version.sdk)"
fi
export ANDROID_API_LEVEL="$API_LEVEL"
export ZMQ_PREFIX="$PREFIX"
pip install -r "$REPO/src/gcs/requirements-termux.txt"

# Prove the native modules load before the first start.
PYTHONPATH="$REPO/src" python -c "import cv2, numpy, orjson, pydantic_core, zmq, pymavlink; print('native modules OK')"
