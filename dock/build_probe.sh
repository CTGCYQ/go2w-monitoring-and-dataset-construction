#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
g++ -O2 -std=c++14 rs_probe.cpp -o rs_probe -lrealsense2 -lpthread
printf 'Built rs_probe (read-only; do not run concurrently with rs_capture).\n'
