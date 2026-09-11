#!/usr/bin/env bash
set -euo pipefail

echo "=== OS ==="
grep -E '^(NAME|VERSION)=' /etc/os-release

echo
echo "=== CUDA ==="
which nvcc
nvcc --version
ls -ld /usr/local/cuda-12.6

python3 - <<'PY'
import ctypes
import os
print("LD_LIBRARY_PATH =", os.environ.get("LD_LIBRARY_PATH", ""))
ctypes.CDLL("libcuda.so")
print("libcuda.so load: OK")
PY

echo
echo "=== PyTorch CUDA ==="
python3 - <<'PY'
import torch
print("torch:", torch.__version__)
print("cuda available:", torch.cuda.is_available())
if torch.cuda.is_available():
    print("device count:", torch.cuda.device_count())
    print("device 0:", torch.cuda.get_device_name(0))
PY

echo
echo "=== ROS Noetic ==="
source /opt/ros/noetic/setup.bash
echo "ROS_DISTRO=$ROS_DISTRO"
which roscore
python3 - <<'PY'
import rospy
print("rospy import: OK")
PY

echo
echo "=== OpenCV ==="
python3 - <<'PY'
import cv2
print("cv2:", cv2.__version__)
PY

echo
echo "=== Catkin ==="
test -f /opt/ros/noetic/setup.bash
echo "/opt/ros/noetic/setup.bash: OK"

echo
echo "Validation complete"
exec /bin/bash
