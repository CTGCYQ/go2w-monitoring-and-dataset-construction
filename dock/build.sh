#!/bin/bash
# Go2-W 深度相机采集程序编译脚本（在机器狗 aarch64 上执行）
set -e
cd "$(dirname "$0")"
echo "=== 编译 rs_capture ==="
g++ -O2 -std=c++14 rs_capture.cpp \
  -o rs_capture \
  -I/usr/include/librealsense2 \
  $(pkg-config --cflags opencv4 2>/dev/null || pkg-config --cflags opencv) \
  -L/usr/lib/aarch64-linux-gnu \
  -lrealsense2 \
  $(pkg-config --libs opencv4 2>/dev/null || pkg-config --libs opencv) \
  -lpthread
echo "=== 编译成功 ==="
ls -la rs_capture
