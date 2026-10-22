#!/usr/bin/env bash
# ==============================================================================
# Off-Target Host Test Runner for acoustic-doa-esp32
# Rapid feature verification & regression prevention without ESP32 hardware
# ==============================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

cd "${ROOT_DIR}"
mkdir -p bin

echo "=== [1/2] Compiling Host Unit Tests with ASAN/UBSAN ==="
g++ -std=gnu++11 -O2 -Wall -Wextra -Werror -fsanitize=address,undefined \
    -I test/mock_esp -I src \
    test/test_adaptive_and_smoothing.cpp \
    src/gcc_phat.cpp \
    src/display_filter.cpp \
    src/post_checker.cpp \
    -o bin/test_host

echo "=== [2/2] Executing Unit Test Suites ==="
./bin/test_host

echo ""
echo "=== All host unit tests passed successfully! ==="
