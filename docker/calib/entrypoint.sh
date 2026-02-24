#!/bin/bash
set -euo pipefail

REPO_ROOT="/workspace/Repos/zed-opencv-calibration"
BUILD_BIN="${REPO_ROOT}/build/stereo_calibration/zed_stereo_calibration"

mkdir -p /captures/svo /captures/previews /captures/png

echo "[calib] Python runtime:"
python3 - <<'PY'
import sys
print(f"python={sys.version.split()[0]}")
try:
    import numpy as np
    print(f"numpy={np.__version__} ({np.__file__})")
except Exception as exc:
    print(f"numpy_error={exc}")
try:
    import cv2
    print(f"cv2={cv2.__version__}")
except Exception as exc:
    print(f"cv2_error={exc}")
PY

if command -v nvidia-smi >/dev/null 2>&1; then
  nvidia-smi >/dev/null 2>&1 && echo "[calib] GPU runtime OK" || echo "[calib] WARNING: nvidia-smi failed"
else
  echo "[calib] WARNING: nvidia-smi not available in container"
fi

if [ ! -f "${BUILD_BIN}" ]; then
  echo "[calib] Building zed-opencv-calibration..."
  mkdir -p "${REPO_ROOT}/build"
  cd "${REPO_ROOT}/build"
  cmake ..
  make -j"$(nproc)"
fi

exec "$@"
