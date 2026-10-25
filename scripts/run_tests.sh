#!/usr/bin/env bash
# ==============================================================================
# Off-Target Host Test Runner for acoustic-doa-esp32
# Rapid feature verification & regression prevention without ESP32 hardware
# ==============================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

cd "${ROOT_DIR}"

echo "=== Running Host Unit Tests via Makefile ==="
make clean
make test

echo ""
echo "=== All host unit tests passed successfully! ==="
