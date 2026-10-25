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

The test harness in [`test/test_adaptive_and_smoothing.cpp`](../test/test_adaptive_and_smoothing.cpp) covers 18 specialized test suites:

1. **`test_gcc_phat_adaptive_noise_floor`**:
   - Verifies the asymmetric attack/decay tracking ($3\text{ dB/s}$ upward creep vs. $30\text{ dB/s}$ fast downward recovery).
   - Validates that sudden sound onsets trigger DoA calculations while steady background noise is adapted away.
2. **`test_display_smoothing_filter`**:
   - Verifies baseline exponential moving average (EMA) response, initial convergence, and silence reset timers ($> 1.5\text{ s}$).
3. **`test_post_checker`**:
   - Validates legacy 2-microphone Power-On Self-Test (POST) flatline, rail short, and stereo bus collision detection.
4. **`test_quad_post_checker`**:
   - Validates 4-microphone health heuristics across all 4 channels (`M0..M3`).
   - Injects flatline ($\le -95\text{ dBFS}$), rail saturation ($\ge -0.8\text{ dBFS}$), intra-bus L/R collisions (Bus 0 and Bus 1), and cross-bus shorts (`GPIO 6` vs `GPIO 7`).
5. **`test_gcc_phat_angle_offset`**:
   - Verifies hardware rotational offset trim compensation and wrap-around arithmetic.
6. **`test_app_config_consistency`**:
   - Verifies central configuration constants: array edge length ($d = 82\text{ mm}$), sample rates ($44100 / 48000\text{ Hz}$), window size locking ($1024$ samples, rejecting 512 and 2048), I2S pin assignments (`BCLK = 5`, `WS = 4`, `DIN0 = 6`, `DIN1 = 7`), and filter cutoff constants.
7. **`test_gcc_phat_dual_band_weights`**:
   - Verifies precomputed raised-cosine frequency weight tables:
     - 4 Edge pairs: $300\text{--}5000\text{ Hz}$ with $100\text{ Hz}$ ramps.
     - 2 Diagonal pairs: $300\text{--}4500\text{ Hz}$ with $100\text{ Hz}$ ramps.
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
12. **`test_rest_binary_batch_serialization`**:
    - Verifies 12-byte header Little-Endian byte packing (version `1`, measurement count, base timestamp, and clock flags).
    - Verifies 20-byte per-record Little-Endian float encoding (`angle_deg`, `delay_us`, calibrated dB, and confidence).
    - Validates buffer overflow protection, homogeneous clock flag validation, and timestamp sequence constraints.
13. **`test_gcc_phat_srp_phat_coarse_grid`**:
    - Verifies 72-direction coarse Steered Response Power ($5^\circ$ step) beamforming across all four quadrants ($0^\circ, \pm 45^\circ, \pm 90^\circ, \pm 135^\circ, \pm 180^\circ, +30^\circ, -60^\circ$), locating the true bearing with zero beamforming error.
14. **`test_gcc_phat_clean_multisource`**:
    - Validates multi-source iterative CLEAN deconvolution with fractional kernel interpolation.
    - Confirms that single-source extraction matches legacy `process()` and extracts two simultaneous sources at $+30^\circ$ and $-60^\circ$.
15. **`test_multi_target_display_tracker`**:
    - Verifies multi-target nearest-neighbor angular association, independent circular smoothing, active silence track pruning ($1500\text{ ms}$ timeout), and persistent `slot_id` identity mapping to preserve needle and chip colors.
16. **`test_gcc_phat_rank_deficiency_and_axis_observability`**:
    - Injects rank-deficient correlation patterns where X-axis or Y-axis pairs drop out.
    - Confirms that 2D rank observability guards (`norm_s_xx`, `norm_s_yy`, `unreg_det`) prevent numerical collapse into phantom $180^\circ$ or $\pm 90^\circ$ bearings.
17. **`test_gcc_phat_split_brain_chimera_fallback`**:
    - Verifies that chimera lag vectors mixing orthogonal talker lags fail closed-triangle consistency.
    - Validates that Pass-0 SRP-PHAT fallback seamlessly recovers the dominant talker and resolves both simultaneous orthogonal speakers ($0^\circ$ and $90^\circ$).
18. **`test_gcc_phat_multisource_edge_cases`**:
    - Validates sub-threshold CLEAN stopping ($< 0.20$), close-angle candidate suppression ($< 30^\circ$ spatial separation gate), and full 3-source extraction (`DOA_MAX_SOURCES = 3`) at $[0.0^\circ, +75.0^\circ, -135.0^\circ]$.

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
| **Off-Target Feature Verification & Regression Prevention** | `make test` or `./scripts/run_tests.sh` | Fast ($<100\text{ ms}$), hardware-free verification of DSP algorithms, WLS solver, health checks, and circular smoothing with ASAN/UBSAN memory guarantees. Runs automatically in CI on every push and PR. |
| **Target Firmware Compilation & Link Verification** | `pio run` | Compiles the full embedded firmware (`src/main.cpp`, Arduino core, FreeRTOS tasks, Octal PSRAM configuration, TFT/BME280 libraries) for the physical ESP32-S3 SoC. Runs in CI after unit tests pass. |
| **Flashing & Live Hardware Debugging** | `pio run -t upload && pio device monitor` | Flashes the firmware to a connected ESP32-S3 board and opens the 115200 baud serial monitor with exception decoding. |

---

## 6. Continuous Integration (CI)

The GitHub Actions workflow in [`.github/workflows/build.yml`](../.github/workflows/build.yml) implements a two-stage automated pipeline triggered on all pushes and pull requests to `main` and `feature/**` branches:

1. **Host Unit Tests (`test` job)**:
   - Executes `make test` under `ubuntu-latest`.
   - Validates all 18 unit test suites with AddressSanitizer and UndefinedBehaviorSanitizer enabled.
   - Fails fast in $<2\text{ s}$ if algorithmic regressions, memory corruption, or boundary violations occur.
2. **PlatformIO Firmware Build (`build` job)**:
   - Depends on the `test` job (`needs: test`).
   - Caches pip and PlatformIO dependencies.
   - Runs `pio run` to verify that the cross-compilation for the `esp32-s3-devkitc-1-n16r8` board links and packages without errors.
