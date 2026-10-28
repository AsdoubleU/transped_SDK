#!/usr/bin/env bash
set -euo pipefail

# Usage:
#   ./run.sh
#   IFACE=enp4s0 ./run.sh

ROOT=$(cd "$(dirname "$0")" && pwd)
BUILD_DIR="$ROOT/build"
EXEC="$BUILD_DIR/EtherCATController"
IFACE=${IFACE:-enp4s0}
MOTORS="$ROOT/config/motors.yaml"
CONTROLLER="$ROOT/config/controller.yaml"

echo "============================================================"
echo " EtherCAT Controller"
echo "============================================================"
echo "EtherCAT        : $IFACE"
echo "Motor YAML      : $MOTORS"
echo "Controller YAML : $CONTROLLER"
echo "============================================================"

cmake -S "$ROOT" -B "$BUILD_DIR" -DCMAKE_BUILD_TYPE=Release
cmake --build "$BUILD_DIR" -j"$(nproc)"
exec sudo -E "$EXEC" "$IFACE" "$MOTORS" "$CONTROLLER"
