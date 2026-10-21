#!/usr/bin/env bash
set -e
cd "$(dirname "${BASH_SOURCE[0]}")"
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# Pin robot DDS to its NIC; do not rely on automatic selection on multi-NIC hosts.
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="enp0s31f6"/></Interfaces></General></Domain></CycloneDDS>'
exec python3 pc_worker.py
