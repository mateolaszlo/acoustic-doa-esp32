# Incremental Implementation Plan: SRP-PHAT + CLEAN Multi-Source DOA

This document outlines the step-by-step vertical slice implementation of **SRP-PHAT + Deconvolving CLEAN** for the `acoustic-doa-esp32` project on the branch `feature/srp-phat-clean-multisource`.

Following the `/incremental-implementation` guidelines, each slice is self-contained, keeps the codebase compilable, includes off-target unit tests (`make test`), and ends with an atomic commit.

---

## Architecture Overview

```
┌─────────────────────────────────────────────────────────────────────────────────────────────┐
│ 4-Mic Audio Block (1024 samples @ 44.1 kHz)                                                 │
└──────────────────────────────────────────────┬──────────────────────────────────────────────┘
                                               │
                                               ▼
┌─────────────────────────────────────────────────────────────────────────────────────────────┐
│ 1. DC Removal & RMS Calculation (Gate Check against Adaptive Noise Floor)                    │
└──────────────────────────────────────────────┬──────────────────────────────────────────────┘
                                               │
                                               ▼
┌─────────────────────────────────────────────────────────────────────────────────────────────┐
│ 2. 4x Radix-2 Forward FFTs -> 6-Pair Wideband Cross-Correlation (300 - 5000 Hz)             │
│    Preserve normalized lag windows [-max_lag, +max_lag] for all 6 pairs on the stack        │
└──────────────────────────────────────────────┬──────────────────────────────────────────────┘
                                               │
                                               ▼
┌─────────────────────────────────────────────────────────────────────────────────────────────┐
│ 3. Iterative CLEAN Loop (Up to DOA_MAX_SOURCES = 3 sources):                                 │
│    ┌───────────────────────────────────────────────────────────────────────────────────┐    │
│    │ Step A: Coarse 72-Angle SRP-PHAT Grid Search (5° step on [-180°, +180°))          │    │
│    │         P(θ) = (1/6) * Σ pair_corr_p[round(τ_p(θ))]                               │    │
│    │         Terminate if peak < correlation_threshold                                 │    │
│    ├───────────────────────────────────────────────────────────────────────────────────┤    │
│    │ Step B: Local 3-Point Parabolic Search (±3 samples around τ_p(θ_seed))            │    │
│    │         Evaluate 4-Triangle Loop-Closure Consistency & 2x2 WLS Azimuth Solver     │    │
│    ├───────────────────────────────────────────────────────────────────────────────────┤    │
│    │ Step C: CLEAN Kernel Subtraction:                                                 │    │
│    │         Subtract bandpass autocorrelation kernel K_p[lag - τ_hat] from pair_corr   │    │
│    │         Enforce angular separation constraint (Δθ >= 25° from prior detections)   │    │
│    └───────────────────────────────────────────────────────────────────────────────────┘    │
└──────────────────────────────────────────────┬──────────────────────────────────────────────┘
                                               │
                                               ▼
┌─────────────────────────────────────────────────────────────────────────────────────────────┐
│ 4. Output MultiDoaResult (Sorted by confidence)                                              │
│    - Sensor/Display: Multi-target tracker matching (DisplaySmoothingFilter array)           │
│    - REST Protocol: Option A v1 binary batch with shared delta_ms timestamps                │
└─────────────────────────────────────────────────────────────────────────────────────────────┘
```

---

## Slices & Checklists

### [x] Slice 1: Configuration Constants & Multi-Source Data Structures
- **Objective**: Define core parameters and return types without altering existing pipeline behavior.
- **Changes**:
  - `src/config/app_config.h`:
    - Added `DOA_MAX_SOURCES = 3`.
    - Added `DOA_MIN_SOURCE_SEPARATION_DEG = 25.0f`.
    - Added `SRP_GRID_ANGLES = 72`, `SRP_GRID_STEP_DEG = 5.0f`, `CLEAN_KERNEL_RADIUS_SAMPLES = 16`.
    - Expanded `NETWORK_BATCH_MAX_MEASUREMENTS = 24` and `NETWORK_QUEUE_LENGTH = 128`.
  - `src/audio/gcc_phat.h`:
    - Defined `struct MultiDoaResult` with up to `DOA_MAX_SOURCES` array.
- **Verification**:
  - `make test`: All 12 unit test suites compile and pass under ASAN/UBSAN with new assertions.
- **Completion Target**: Build succeeds, data structures defined, backwards compatibility maintained.

---

### [x] Slice 2: Wideband Weight Tables & Autocorrelation Kernel Generation
- **Objective**: Precompute bandpass autocorrelation kernels for clean time-domain subtraction in CLEAN.
- **Changes**:
  - `src/config/app_config.h`:
    - Raised `GCC_PHAT_EDGE_MAX_HZ = 5000.0f` and `GCC_PHAT_DIAG_MAX_HZ = 4500.0f` for wideband peak resolution.
  - `src/audio/gcc_phat.h` / `src/audio/gcc_phat.cpp`:
    - Added member kernel buffers `clean_kernel_edge` and `clean_kernel_diag` with zero heap allocation.
    - Implemented `precomputeCleanKernels()` computing normalized IFFT kernels during `init()`.
  - `test/test_adaptive_and_smoothing.cpp`:
    - Added unit test assertions verifying kernel normalization ($K[0] = 1.0$), conjugate symmetry ($K[+l] == K[-l]$), and smooth roll-off.
    - Updated synthetic multitone generator to span the full wideband spectrum (260 Hz – 4900 Hz).
- **Verification**:
  - `make test`: All 12 unit test suites compile and pass under ASAN/UBSAN with improved WLS angular precision (< 0.01° error).
- **Completion Target**: Kernels dynamically generated during `init()`, memory leaks checked with AddressSanitizer.

---

### [x] Slice 3: Lag Window Preservation & 72-Point SRP-PHAT Grid Search
- **Objective**: Preserve normalized correlation lags for all 6 pairs on the stack and implement fast 72-point angular beamforming.
- **Changes**:
  - `src/config/app_config.h`:
    - Added `PAIR_CORR_RADIUS = 22` and `PAIR_CORR_SIZE = 45`.
  - `src/audio/gcc_phat.h` / `src/audio/gcc_phat.cpp`:
    - Implemented `populatePairCorrelations()` extracting normalized correlation lags `[-22, +22]` for all 6 pairs.
    - Implemented `evaluateSrpPhat(pair_corr, best_angle, best_power)` evaluating 72 angles ($5^\circ$ grid) in $<10\ \mu\text{s}$ CPU time.
    - Updated `process()` to preserve `pair_corr` on the stack.
  - `test/test_adaptive_and_smoothing.cpp`:
    - Added 13th unit test suite `test_gcc_phat_srp_phat_coarse_grid` verifying coarse angle localization across cardinal and oblique directions.
- **Verification**:
  - `make test`: All 13 unit test suites pass under ASAN/UBSAN with zero coarse grid error ($0.0^\circ$ error, power $>0.96$).
- **Completion Target**: Coarse spatial spectrum correctly locates sound seeds under $< 10\ \mu\text{s}$ CPU budget.

---

### [x] Slice 4: CLEAN Kernel Subtraction Loop & Multi-Source Parabolic WLS
- **Objective**: Extract up to 3 simultaneous acoustic sources using iterative deconvolution and closed-triangle loop-closure WLS.
- **Changes**:
  - `src/audio/gcc_phat.h` / `src/audio/gcc_phat.cpp`:
    - Implemented `GccPhat::processMulti(ch0, ch1, ch2, ch3)` returning `MultiDoaResult`.
    - Maintained `GccPhat::process()` as 100% backwards-compatible wrapper returning primary source.
    - Added `solveWlsAzimuth(..., min_confidence = -1.0f)` allowing adaptive confidence thresholds for secondary/tertiary CLEAN passes.
    - Iterative CLEAN loop: Pass 0 extracts dominant source via WLS and subtracts acoustic imprint; Pass 1..2 evaluate SRP-PHAT on remaining lag field, refine candidates with 3-point parabolic interpolation, verify loop-closure consistency, and enforce minimum angular separation ($\ge 25^\circ$).
  - `test/test_adaptive_and_smoothing.cpp`:
    - Added `test_gcc_phat_clean_multisource()` verifying both single-source backwards-compatibility and two simultaneous uncorrelated sources (+30.0° and -60.0°).
- **Verification**:
  - `make test`: All 14 unit test suites compile and pass under ASAN/UBSAN with zero errors and zero memory leaks.
- **Completion Target**: Multi-source extraction working natively in host tests.

---

### [x] Slice 5: Binary REST Serialization Protocol (Option A)
- **Objective**: Support simultaneous multi-target uploads sharing the same frame timestamp without breaking existing backend decoders.
- **Changes**:
  - `src/network/rest_protocol.h`:
    - Verified `serialize_measurement_batch()` handles batches of up to 24 measurements with identical `timestamp_ms` values ($\Delta t = 0$).
  - `test/test_adaptive_and_smoothing.cpp`:
    - Added unit test creating a 24-record simultaneous multi-source batch (8 frames * 3 simultaneous sources).
    - Confirmed serializer accepts them, packages valid 20-byte records, and decodes with identical `delta_ms = 0` for simultaneous detections in the same frame.
- **Verification**:
  - `make test`: All 14 unit test suites pass under ASAN/UBSAN with zero errors.
- **Completion Target**: Protocol validated for multi-target batches without bumping version number or modifying decoder contract.

---

### [ ] Slice 6: Multi-Target Radar UI & Tracker Association
- **Objective**: Render multiple simultaneous radar needles on the ST7735 display with independent $S^1$ smoothing filters.
- **Changes**:
  - `src/ui/display_filter.h` / `src/ui/display_filter.cpp`:
    - Add multi-source filter bank: `DisplaySmoothingFilter filters[DOA_MAX_SOURCES]`.
    - Implement nearest-neighbor angle association using `shortestAngularDiff()`.
  - `src/ui/radar_ui.h` / `src/ui/radar_ui.cpp`:
    - Update `drawRadar()` to accept up to 3 angles/confidences and draw primary (solid bright), secondary, and tertiary (dimmed/stippled) needles.
  - `src/main.cpp`:
    - Update `sensor_display_task` to receive `MultiDoaResult` from `display_queue` and update the filter bank.
- **Verification**:
  - Unit test verifying multi-filter tracking and association (independent tracking of $0^\circ$ and $90^\circ$ without cross-interference).
  - `make test` passes.
- **Completion Target**: UI capable of rendering 1, 2, or 3 needles concurrently with stable smoothing.

---

### [ ] Slice 7: End-to-End Main Pipeline Integration, Documentation & CI
- **Objective**: Wire `GccPhat::processMulti()` into `dsp_task` on Core 1 and document the new capability.
- **Changes**:
  - `src/main.cpp`:
    - Call `processMulti()` in `dsp_task`.
    - Dispatch each detected source to `network_queue` and `display_queue`.
  - `docs/firmware.md` & `README.md`:
    - Document SRP-PHAT + CLEAN algorithm, mathematical formulation, and multi-source tracking parameters.
  - Run GitHub Actions CI to verify cross-compilation with PlatformIO for the physical ESP32-S3 board.
- **Verification**:
  - `make test` (all unit tests pass under ASAN/UBSAN).
  - Git commit & push to `feature/srp-phat-clean-multisource`.
  - GitHub Actions CI workflow finishes green (`test` and `build` jobs pass).
- **Completion Target**: Fully functional, verified, and documented multi-source direction estimator.

---

## Progress Summary Table

| Slice | Description | Status | Commit SHA |
|---|---|:---:|:---:|
| **Slice 1** | Configuration Constants & Multi-Source Data Structures | [x] Completed | `fef9bf4` |
| **Slice 2** | Wideband Weight Tables & Autocorrelation Kernels | [x] Completed | `a18ef55` |
| **Slice 3** | Lag Window Preservation & 72-Point SRP-PHAT Grid Search | [x] Completed | `0e63c9b` |
| **Slice 4** | CLEAN Deconvolution Loop & Multi-Source Parabolic WLS | [x] Completed | `706b132` |
| **Slice 5** | Binary REST Batch Capacity & Protocol Verification | [x] Completed | In progress |
| **Slice 6** | Multi-Target Radar UI & Filter Bank Association | [ ] Pending | — |
| **Slice 7** | System Integration, Documentation & CI Verification | [ ] Pending | — |
