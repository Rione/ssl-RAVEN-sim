#!/usr/bin/env bash
# Linux (Ubuntu 24.04+) build script for ssl-RAVEN-Sim
# Tested on Ubuntu 24.04 LTS with system Qt 6.4.2
#
# Prerequisites (installed automatically by this script):
#   qt6-quick3d-dev qt6-quick3dphysics-dev
#   libprotobuf-dev protobuf-compiler
#   libboost-dev ninja-build cmake

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="$SCRIPT_DIR/build-linux"

# ── Install dependencies ──────────────────────────────────────────────────
echo "=== Checking / installing dependencies ==="
MISSING=()
for pkg in \
    qt6-quick3d-dev \
    qt6-quick3dphysics-dev \
    qt6-shadertools-dev \
    libprotobuf-dev \
    protobuf-compiler \
    libboost-dev \
    ninja-build \
    cmake; do
    dpkg -s "$pkg" &>/dev/null || MISSING+=("$pkg")
done

if [ ${#MISSING[@]} -gt 0 ]; then
    echo "Installing: ${MISSING[*]}"
    sudo apt-get update -qq
    sudo apt-get install -y "${MISSING[@]}"
fi

# ── CMake configure ───────────────────────────────────────────────────────
echo "=== CMake configure ==="
cmake -G Ninja \
    -DCMAKE_BUILD_TYPE=Release \
    -S "$SCRIPT_DIR" \
    -B "$BUILD_DIR"

# ── Build ─────────────────────────────────────────────────────────────────
echo "=== Building ==="
cmake --build "$BUILD_DIR" --parallel "$(nproc)"

echo ""
echo "=== Build succeeded! ==="
echo "Executable: $BUILD_DIR/bin/m2-Sim"
echo ""
echo "Run with:"
echo "  $BUILD_DIR/bin/m2-Sim"
