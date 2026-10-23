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

### [ ] Slice 2: Wideband Weight Tables & Autocorrelation Kernel Generation
- **Objective**: Precompute bandpass autocorrelation kernels for clean time-domain subtraction in CLEAN.
- **Changes**:
  - `src/audio/gcc_phat.h` / `src/audio/gcc_phat.cpp`:
    - Add precomputed kernel buffer `float* clean_kernel_edge` and `float* clean_kernel_diag`.
    - In `init()` / `allocateBuffers()`:
      - Compute normalized IFFT of the raised-cosine frequency weighting window to produce the zero-lag centered kernel.
      - Store normalized kernel values for lag offsets `[-CLEAN_KERNEL_RADIUS, +CLEAN_KERNEL_RADIUS]`.
- **Verification**:
  - Add unit test in `test/test_adaptive_and_smoothing.cpp` verifying kernel symmetry, peak at lag 0 ($K[0] = 1.0$), and fast roll-off to zero.
  - `make test` passes.
- **Completion Target**: Kernels dynamically generated during `init()`, memory leaks checked with AddressSanitizer.

---

### [ ] Slice 3: Short-Lag Window Preservation & Coarse SRP-PHAT Grid Search
- **Objective**: Preserve normalized correlation lags for all 6 pairs on the stack and implement fast 72-point angular beamforming.
- **Changes**:
  - `src/audio/gcc_phat.h` / `src/audio/gcc_phat.cpp`:
    - Implement `evaluateSrpPhat(const float pair_corr[6][45], float &best_angle, float &best_power)`.
    - In `GccPhat::processMulti()`:
      - For each of the 6 pairs, copy normalized real correlation values within physical bounds `[-max_lag, +max_lag]` into a stack array `float pair_corr[6][45]`.
      - Evaluate theoretical delay table $\tau_p(\theta) = \frac{d(\text{dir\_x}_p \sin\theta + \text{dir\_y}_p \cos\theta)}{c} f_s$ across 72 angles ($5^\circ$ resolution).
- **Verification**:
  - Add unit test verifying SRP-PHAT finds synthetic single-source incident angles ($0^\circ, 45^\circ, 90^\circ, 180^\circ$) within $\pm 5^\circ$ before WLS refinement.
  - `make test` passes.
- **Completion Target**: Coarse spatial spectrum correctly locates sound seeds under $< 10\ \mu\text{s}$ CPU budget.

---

### [ ] Slice 4: CLEAN Kernel Subtraction Loop & Multi-Source Parabolic WLS
- **Objective**: Extract up to 3 simultaneous acoustic sources using iterative deconvolution and closed-triangle loop-closure WLS.
- **Changes**:
  - `src/audio/gcc_phat.cpp`:
    - Implement `GccPhat::processMulti(ch0, ch1, ch2, ch3)`:
      1. For $k = 0 \dots \text{DOA\_MAX\_SOURCES} - 1$:
         a. Find $\theta_{\text{seed}}$ using `evaluateSrpPhat`. Stop if $P(\theta_{\text{seed}}) < \text{threshold}$.
         b. Refine peak per-pair in narrow window ($\pm 3$ samples around $\tau_p(\theta_{\text{seed}})$) with 3-point parabolic interpolation.
         c. Run `solveWlsAzimuth()` for Source $k$.
         d. Verify angular separation $\ge 25^\circ$ from all previously accepted sources $0 \dots k-1$.
         e. Subtract scaled autocorrelation kernel $K_p[\tau - \hat{\tau}_{p,k}]$ from `pair_corr[p]`.
    - Keep `GccPhat::process()` as a 100% backwards-compatible wrapper returning `multi_res.sources[0]`.
- **Verification**:
  - Add unit test injecting two simultaneous synthetic plane waves (e.g. Source 1 at $+30^\circ$, Source 2 at $-60^\circ$). Verify both angles are accurately resolved with $< 1^\circ$ error.
  - Inject single loud source; verify CLEAN does not produce a ghost source within $25^\circ$.
  - `make test` passes with ASAN/UBSAN.
- **Completion Target**: Multi-source extraction working natively in host tests with sub-degree accuracy.

---

### [ ] Slice 5: Binary REST Serialization Protocol (Option A)
- **Objective**: Support simultaneous multi-target uploads sharing the same frame timestamp without breaking existing backend decoders.
- **Changes**:
  - `src/network/rest_protocol.h`:
    - Verify `serialize_measurement_batch()` handles batches of up to 24 measurements with identical `timestamp_ms` values ($\Delta t = 0$).
  - `test/test_adaptive_and_smoothing.cpp`:
    - Add unit test creating a 3-source simultaneous measurement batch (all with identical timestamps).
    - Confirm serializer accepts them, packages valid 20-byte records, and decodes with identical `delta_ms = 0`.
- **Verification**:
  - `make test` passes.
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
| **Slice 1** | Configuration Constants & Multi-Source Data Structures | [x] Completed | `eba0c38` |
| **Slice 2** | Wideband Weight Tables & Autocorrelation Kernels | [ ] Pending | — |
| **Slice 3** | Lag Window Preservation & 72-Point SRP-PHAT Grid Search | [ ] Pending | — |
| **Slice 4** | CLEAN Deconvolution Loop & Multi-Source Parabolic WLS | [ ] Pending | — |
| **Slice 5** | Binary REST Batch Capacity & Protocol Verification | [ ] Pending | — |
| **Slice 6** | Multi-Target Radar UI & Filter Bank Association | [ ] Pending | — |
| **Slice 7** | System Integration, Documentation & CI Verification | [ ] Pending | — |
