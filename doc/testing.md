# Off-Target Host Testing & Feature Verification

This document describes the host-based unit testing and algorithmic verification architecture for the `acoustic-doa-esp32` project.

---

## 1. Overview & Purpose

Because embedded microcontrollers require physical wiring, flashing, and serial monitoring, relying solely on on-device testing slows down feature development and makes continuous integration difficult.

To enable **rapid feature verification** and **prevent functional regression** without physical ESP32 hardware:
- All core DSP mathematics, signal processing pipelines, circular filtering algorithms, and hardware health heuristics are decoupled from hardware-specific peripherals.
- An off-target host test harness executes natively on host Linux and macOS systems under `g++` or `clang++`.
- Tests run with **AddressSanitizer (ASAN)** and **UndefinedBehaviorSanitizer (UBSAN)** enabled, detecting memory corruption, out-of-bounds array reads, use-after-free, memory leaks, and undefined floating-point math that would otherwise cause silent crashes on the microcontroller.
- Complete execution completes in approximately **100 milliseconds**.

---

## 2. Mock Architecture (`test/mock_esp/`)

When running off-target, the tests link against lightweight, bit-exact mocks located in [`test/mock_esp/`](../test/mock_esp/):

| Mock Header | Purpose |
|---|---|
| [`esp_dsp.h`](../test/mock_esp/esp_dsp.h) | Bit-exact implementation of the Cooley-Tukey Radix-2 Complex Fast Fourier Transform (`dsps_fft2r_fc32`), bit reversal (`dsps_bit_rev_fc32`), and Hann window generation (`dsps_wind_hann_f32`), exactly reproducing the ESP32-S3 assembly DSP behavior. |
| [`esp_heap_caps.h`](../test/mock_esp/esp_heap_caps.h) | Emulates ESP-IDF's dual internal SRAM (`MALLOC_CAP_INTERNAL`) and Octal PSRAM (`MALLOC_CAP_SPIRAM`) allocators, allowing unit tests to inject memory allocation failures and verify graceful fallback and rollback. |
| [`esp_log.h`](../test/mock_esp/esp_log.h) | Replaces ESP-IDF logging macros (`ESP_LOGE`, `ESP_LOGW`, `ESP_LOGI`) with standard `printf` formatting. |

---

## 3. Test Suites Overview

The test harness in [`test/test_adaptive_and_smoothing.cpp`](../test/test_adaptive_and_smoothing.cpp) covers 11 specialized test suites:

1. **`test_gcc_phat_adaptive_noise_floor`**:
   - Verifies the asymmetric attack/decay tracking ($3\text{ dB/s}$ upward creep vs. $30\text{ dB/s}$ fast downward recovery).
   - Validates that sudden sound onsets trigger DoA calculations while steady background noise is adapted away.
2. **`test_display_smoothing_filter`**:
   - Verifies baseline exponential moving average (EMA) response, initial convergence, and silence reset timers ($> 1.5\text{ s}$).
3. **`test_post_checker`**:
   - Validates legacy 2-microphone Power-On Self-Test (POST) flatline, rail short, and stereo bus collision detection.
4. **`test_quad_post_checker`**:
   - Validates 4-microphone health heuristics across all 4 channels (`M0..M3`).
   - Injects flatline ($\le -95\text{ dBFS}$), rail saturation ($\ge -3\text{ dBFS}$), intra-bus L/R collisions (Bus 0 and Bus 1), and cross-bus shorts (`GPIO 6` vs `GPIO 7`).
5. **`test_gcc_phat_angle_offset`**:
   - Verifies hardware rotational offset trim compensation and wrap-around arithmetic.
6. **`test_app_config_consistency`**:
   - Verifies central configuration constants: array edge length ($d = 82\text{ mm}$), sample rates ($44100 / 48000\text{ Hz}$), window sizes ($512 / 1024 / 2048$), I2S pin assignments (`BCLK = 5`, `WS = 4`, `DIN0 = 6`, `DIN1 = 7`), and filter cutoff constants.
7. **`test_gcc_phat_dual_band_weights`**:
   - Verifies precomputed raised-cosine frequency weight tables:
     - 4 Edge pairs: $300\text{--}2200\text{ Hz}$ with $100\text{ Hz}$ ramps.
     - 2 Diagonal pairs: $300\text{--}1450\text{ Hz}$ with $100\text{ Hz}$ ramps (rolling off before diagonal spatial aliasing at $1483\text{ Hz}$).
8. **`test_gcc_phat_quad_rms_and_pipeline`**:
   - Verifies 4-channel omnidirectional unwindowed RMS symmetry across all inputs and buffer rollback if allocation fails.
9. **`test_gcc_phat_quad_wls_all_quadrants`**:
   - Injects synthetic delayed broadband signals across all 4 quadrants ($0^\circ, +45^\circ, +90^\circ, +135^\circ, \pm 180^\circ, -135^\circ, -90^\circ, -45^\circ$, etc.).
   - Confirms that the $2\times 2$ Weighted Least-Squares (WLS) solver reconstructs the true bearing with $< 0.02^\circ$ error.
10. **`test_gcc_phat_loop_closure_multipath_rejection`**:
    - Injects severe multipath corruption (+8 sample lag spike) into one microphone pair.
    - Confirms that the 4-triangle loop closure filter ($e_0 \dots e_3$) penalizes the corrupted pair with a Gaussian weight $\psi_p$ and recovers the true angle.
11. **`test_display_circular_smoothing_and_outliers`**:
    - Verifies circular geodesic smoothing on the unit circle $S^1$.
    - Tests smooth crossing across the $\pm 180^\circ$ back boundary ($+175^\circ \leftrightarrow -175^\circ$) without swinging through $0^\circ$.
    - Confirms 2-frame outlier candidate rejection across the boundary.

---

## 4. How to Run Tests

### Method 1: Using `make` (Recommended)
Run all unit tests with ASAN/UBSAN:
```bash
make test
```
Clean build artifacts:
```bash
make clean
```

### Method 2: Using the Shell Script
Run the automated runner script:
```bash
./scripts/run_tests.sh
```

### Method 3: Direct Compilation
Compile and run directly using `g++` or `clang++`:
```bash
mkdir -p bin
g++ -std=gnu++11 -O2 -Wall -Wextra -Werror -fsanitize=address,undefined \
    -I test/mock_esp -I src \
    test/test_adaptive_and_smoothing.cpp \
    src/audio/gcc_phat.cpp \
    src/ui/display_filter.cpp \
    src/audio/post_checker.cpp \
    -o bin/test_host

./bin/test_host
```

---

## 5. Host Testing vs. PlatformIO Firmware Builds

| Task | Tool / Command | Purpose |
|---|---|---|
| **Off-Target Feature Verification & Regression Prevention** | `make test` or `./scripts/run_tests.sh` | Fast ($<100\text{ ms}$), hardware-free verification of DSP algorithms, WLS solver, health checks, and circular smoothing with ASAN/UBSAN memory guarantees. |
| **Target Firmware Compilation & Link Verification** | `pio run` | Compiles the full embedded firmware (`src/main.cpp`, Arduino core, FreeRTOS tasks, Octal PSRAM configuration, TFT/BME280 libraries) for the physical ESP32-S3 SoC. |
| **Flashing & Live Hardware Debugging** | `pio run -t upload && pio device monitor` | Flashes the firmware to a connected ESP32-S3 board and opens the 115200 baud serial monitor with exception decoding. |
