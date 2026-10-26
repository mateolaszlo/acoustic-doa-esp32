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
| [`Adafruit_GFX.h`](../test/mock_esp/Adafruit_GFX.h) & [`Adafruit_ST7735.h`](../test/mock_esp/Adafruit_ST7735.h) | Lightweight frame buffer and graphics primitive mocks for off-target ST7735 TFT radar and prompt rendering verification. |
| [`SPI.h`](../test/mock_esp/SPI.h) | Host mock for SPI bus communication. |
| [`Arduino.h`](../test/mock_esp/Arduino.h) & [`Wire.h`](../test/mock_esp/Wire.h) | Minimal Arduino core and I2C TwoWire mocks providing `HardwareSerial Serial` output capture and I2C bus probing for off-target telemetry verification. |

---

## 3. Test Suites Overview

The test harness in [`test/test_adaptive_and_smoothing.cpp`](../test/test_adaptive_and_smoothing.cpp) covers 23 specialized test suites:

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
19. **`test_gcc_phat_phase_one_memory_safety_and_threshold_gates`**:
    - Verifies lag bounds clamping (`PAIR_CORR_RADIUS`) preventing stack buffer out-of-bounds reads (`CRIT-01`).
    - Validates Pass-0 primary fallback correlation threshold gating (`CRIT-02`).
    - Validates null pointer and out-of-bounds geometry memory safety under AddressSanitizer and UndefinedBehaviorSanitizer.
20. **`test_phase_two_security_and_contracts`**:
    - Verifies `DeviceConfig` bounds checking, float `NaN`/`Inf` rejection, and sanitization (`MED-02`).
    - Verifies HTML attribute escaping for SSID/URL strings preventing injection and XSS (`MED-02`).
    - Validates multi-source confidence descending sorting, primary diagnostic preservation, and POST flatline diagnosis (`MED-03`).
21. **`test_phase_three_responsiveness_and_expanded_harness`**:
    - Verifies decoupled active track hold ($250\text{ ms}$) vs. EMA filter retention ($1500\text{ ms}$) and inactive slot reclamation (`MED-01`).
    - Validates rapid intermittent sound bursts at $240\text{ ms}$ intervals across alternating angles without jitter or track deallocation.
    - Validates `RadarDisplay` phosphor blip decay ($500\text{ ms}$), duplicate blip suppression, multi-needle rendering, and degraded hardware alert screens.
    - Validates tracker and UI robustness against non-finite floats (`NaN`, `Inf`) and clock wrap / negative timestamps.
    - Validates expanded array geometries ($d > 0.12\text{ m}$, up to $0.20\text{ m}$) under ASAN/UBSAN with zero stack/heap over-reads.
    - Confirms rejection of single sources with confidence in $[0.20, \text{correlation\_threshold})$.
    - Verifies non-monotonic batch timestamp clamping and server UTC clock offset stability.
22. **`test_phase_two_serial_ui_and_logging`**:
    - Validates `SerialUI` serial telemetry formatting across single- and multi-source DOA events (`Detected angle: ...`).
    - Verifies power-on self-test ASCII report generation for nominal and hardware-degraded conditions.
    - Validates critical hardware alert banners, I2C bus scanning diagnostics, and rate-limited DSP performance telemetry.
23. **`test_phase_four_localization_and_docs_contracts`**:
    - Validates 100% English console output across `SerialUI` diagnostics with zero legacy Slovenian tokens.
    - Enforces binary REST protocol contracts: 12-byte header, 20-byte record, and 492-byte max batch payload sizing.
    - Validates IEEE-754 binary32 angle packing precision across the full-circle $\pm 180.0^\circ$ azimuth boundary.
    - Validates `DeviceConfig` contract compatibility against the documented REST API calibration schema.
24. **`test_wifi_and_api_serial_logging`**:
    - Validates Wi-Fi disconnect reason code decoding (`NO_AP_FOUND = 201`, `AUTH_FAIL = 202`, `4WAY_HANDSHAKE_TIMEOUT = 15`, `HANDSHAKE_TIMEOUT = 204`, `BEACON_TIMEOUT = 200`, and unknown reason codes).
    - Validates anti-spam throttling for repetitive Wi-Fi disconnections (15-second throttle window with attempt counts).
    - Validates REST API wrong API key detection (HTTP 401 Unauthorized / HTTP 403 Forbidden) across calibration and measurement endpoints.
    - Validates HTTP 404 (device/route not found), HTTP 500, and network timeout classification.
    - Validates measurement batch streaming anti-spam throttling (immediate initial active print, suppressed intermediate 5 Hz batches, milestone summary at batch 50, and 15-second error throttling).

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
g++ -std=gnu++11 -O1 -Wall -Wextra -Werror -fsanitize=address,undefined,float-divide-by-zero \
    -I test/mock_esp -I src \
    test/test_adaptive_and_smoothing.cpp \
    src/audio/gcc_phat.cpp \
    src/ui/display_filter.cpp \
    src/audio/post_checker.cpp \
    src/ui/radar_ui.cpp \
    src/ui/serial_ui.cpp \
    -o bin/test_host

./bin/test_host
```

---

## 5. Host Testing vs. PlatformIO Firmware Builds

| Task | Tool / Command | Purpose |
|---|---|---|
| **Off-Target Feature Verification & Regression Prevention** | `make test` or `./scripts/run_tests.sh` | Fast ($<100\text{ ms}$), hardware-free verification of DSP algorithms, WLS solver, health checks, radar display rendering, and circular smoothing with ASAN/UBSAN memory guarantees. Runs automatically in CI on every push and PR. |
| **Target Firmware Compilation & Link Verification** | `pio run` | Compiles the full embedded firmware (`src/main.cpp`, Arduino core, FreeRTOS tasks, Octal PSRAM configuration, TFT/BME280 libraries) for the physical ESP32-S3 SoC. Runs in CI after unit tests pass. |
| **Flashing & Live Hardware Debugging** | `pio run -t upload && pio device monitor` | Flashes the firmware to a connected ESP32-S3 board and opens the 115200 baud serial monitor with exception decoding. |

---

## 6. Continuous Integration (CI)

The GitHub Actions workflow in [`.github/workflows/build.yml`](../.github/workflows/build.yml) implements a two-stage automated pipeline triggered on all pushes and pull requests to `main` and `feature/**` branches:

1. **Host Unit Tests (`test` job)**:
   - Executes `make test` under `ubuntu-latest`.
   - Validates all 23 unit test suites with AddressSanitizer and UndefinedBehaviorSanitizer enabled.
   - Fails fast in $<2\text{ s}$ if algorithmic regressions, memory corruption, or boundary violations occur.
2. **PlatformIO Firmware Build (`build` job)**:
   - Depends on the `test` job (`needs: test`).
   - Caches pip and PlatformIO dependencies.
   - Runs `pio run` to verify that the cross-compilation for the `esp32-s3-devkitc-1-n16r8` board links and packages without errors.
