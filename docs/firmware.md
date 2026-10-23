# Firmware architecture

## Source files

| File | Responsibility |
|---|---|
| `src/main.cpp` | FreeRTOS tasks orchestration (`dsp_task`, `sensor_display_task`, `network_task`), peripheral setup, and system initialization |
| `src/config/app_config.h` | Central hardware pinout, 4-mic topology, dual-band DSP cutoffs, FreeRTOS task, and radar UI constants |
| `src/config/device_config.h` | Device configuration data structures, network settings, acoustic threshold parameters, and calibration values |
| `src/config/config_store.h` / `.cpp` | Persistent NVS storage load/save routines via ESP32 `Preferences` API |
| `src/audio/gcc_phat.h` / `.cpp` | 4-channel FFT pipeline, dual-band raised-cosine spectral weighting, 6-pair GCC-PHAT, triangle loop-closure validation, and $2\times 2$ WLS 2D azimuth solver |
| `src/audio/i2s_sampler.h` / `.cpp` | Synchronized dual-I2S (`I2S0` Master RX + `I2S1` Slave RX via GPIO Matrix) driver installation and 4-channel quad DMA reads (`read_i2s_quad`) |
| `src/audio/post_checker.h` / `.cpp` | Power-On Self-Test (POST) 4-microphone health heuristics (`evaluate_quad_microphone_health`) and bus collision detection |
| `src/network/rest_protocol.h` | Hardware-independent 12-byte header / 20-byte record binary batch serialization format and little-endian packing |
| `src/network/rest_client.h` / `.cpp` | HTTP keep-alive batch transmission (`POST /api/v1/meritve`) and remote calibration retrieval (`GET /api/v1/naprave/{id}/kalibracija`) |
| `src/network/setup_portal.h` / `.cpp` | SoftAP captive portal and embedded HTTP web server for device Wi-Fi and calibration provisioning |
| `src/ui/radar_ui.h` / `.cpp` | 360° circular radar scope rendering, polar needle visualization, POST checklist prompt, degraded screen, and setup portal UI on ST7735 TFT |
| `src/ui/serial_ui.h` / `.cpp` | Human-facing serial console TLI formatting: ASCII boot self-test report card, degraded alert banner, I2C bus scan, live DOA event telemetry, and DSP metrics |
| `src/ui/display_filter.h` / `.cpp` | Circular $S^1$ EMA smoothing (`wrapAngle180`, `shortestAngularDiff`), outlier rejection across $\pm 180^\circ$, and radar needle angle calculation |
| `platformio.ini` | Board, framework, monitor, and library dependencies |

## Audio processing (4-Microphone Square Array & 6-Pair WLS Fusion)

The DSP task reads a synchronized 4-channel audio block via `read_i2s_quad()`. The default is 1024 samples at 44100 Hz. Supported sample rates are 44100 and 48000 Hz; supported FFT/window sizes are 512, 1024, and 2048 samples. The four microphones form a horizontal planar square of side length $d = 0.082\text{ m}$ (`Mic 0` Right-Back, `Mic 1` Right-Front, `Mic 2` Left-Front, `Mic 3` Left-Back). Each block is processed as follows:

1. **DC Offset Removal & Omnidirectional RMS**: Subtract the per-channel mean in 64-bit integer precision (`int64_t`), convert to normalized floating-point values, and compute the unwindowed 4-channel omnidirectional RMS level (`0.25f * sum(raw_s[0..3]^2)`).
2. **Adaptive Noise Floor Gating**: Evaluate the block RMS against the effective threshold (adaptive noise floor + SNR margin, or fixed threshold). Quiet frames return early without running FFTs.
3. **Windowing & 4 Forward FFTs**: Apply a Hann window to each of the 4 channels and compute 4 in-place radix-2 complex forward FFTs (`dsps_fft2r_fc32` + `dsps_bit_rev_fc32`) yielding $X_0(f) \dots X_3(f)$.
4. **6-Pair Dual-Band PHAT Cross-Correlation**: Iterate over all $\binom{4}{2} = 6$ microphone pairs:
   - **Edge Pairs ($p \in \{0, 1, 2, 3\}$, baseline $d = 82\text{ mm}$)**: Weighted by precomputed `band_weight_edge` ($300\text{--}2200\text{ Hz}$ with $100\text{ Hz}$ raised-cosine ramps).
   - **Diagonal Pairs ($p \in \{4, 5\}$, baseline $d\sqrt{2} \approx 116\text{ mm}$)**: Weighted by precomputed `band_weight_diag` ($300\text{--}1450\text{ Hz}$ with $100\text{ Hz}$ raised-cosine ramps) to roll off cleanly below the diagonal spatial aliasing frequency ($f_{\text{alias}} = \frac{c}{2d\sqrt{2}} \approx 1483\text{ Hz}$).
   - Compute PHAT-normalized cross-spectra in the shared `ccorr` buffer, run the inverse FFT, search within each pair's physical lag bound $\lceil \frac{\|\mathbf{b}_p\|}{c} f_s \rceil + 1$, and refine the peak lag $\tau_p$ and peak $\text{confidence}_p \in [0, 1]$ via 3-point parabolic interpolation.
5. **Closed-Triangle Loop-Closure Consistency**: Evaluate the 4 closed spatial triangles ($\tau_{ij} + \tau_{jk} + \tau_{ki} = 0$) across the 4-mic square and assign each pair a Gaussian consistency weight $\psi_p = \exp\left(-e_{\text{pair},p}^2 / (2\sigma_{\text{loop}}^2)\right)$ with $\sigma_{\text{loop}} = 1.5\text{ samples}$ to suppress single-pair multipath reflections.
6. **$2\times 2$ Weighted Least-Squares (WLS) 2D Azimuth Solver**:
   - Solve the regularized $2\times 2$ normal equations $\mathbf{A}\hat{\mathbf{u}} = \mathbf{g}$ with composite pair weights $w_p = \text{confidence}_p^2 \cdot \psi_p$ for the unit propagation vector $\hat{\mathbf{u}} = [\hat{u}_x, \hat{u}_y]^T$.
   - Compute full $360^\circ$ azimuth clockwise from North/Front ($+Y = 0^\circ$, $+X = +90^\circ$, $\pm 180^\circ = -Y$, $-X = -90^\circ$):
     $$\phi_{\text{raw}} = \text{atan2f}(\hat{u}_x, \hat{u}_y) \cdot \frac{180^\circ}{\pi} \in [-180^\circ, +180^\circ]$$
   - Apply rotational zero-angle offset compensation with circular wrap-around into $[-180^\circ, +180^\circ]$:
     $$\phi_{\text{cal}} = \text{wrap}_{180}(\phi_{\text{raw}} - \phi_{\text{offset}})$$
   - Report equivalent projected broadside delay along the X-axis baseline (`delay_us`):
     $$\tau_{\text{proj}} = \left(\frac{d \cdot \sin(\phi_{\text{cal}} \cdot \pi / 180^\circ)}{c}\right) \times 10^6 \quad [\mu\text{s}]$$

### Multi-Source Estimation: SRP-PHAT + Deconvolving CLEAN

To track up to `DOA_MAX_SOURCES = 3` concurrent acoustic events within a single 1024-sample audio block without spatial aliasing or destructive mutual interference, the firmware combines wideband Steered Response Power (SRP-PHAT) with iterative CLEAN deconvolution and 2D WLS refinement:

1. **Wideband Correlation Kernels**:
   - Edge pairs are filtered up to $5000\text{ Hz}$ (`GCC_PHAT_EDGE_MAX_HZ`) and diagonal pairs up to $4500\text{ Hz}$ (`GCC_PHAT_DIAG_MAX_HZ`).
   - Increasing the bandwidth shrinks the time-domain correlation peak width ($\Delta\tau \approx \frac{f_s}{2f_{\max}}$) from $\sim 12\text{ samples}$ down to $\sim 3\text{ samples}$, enabling multiple distinct correlation lobes to coexist within the $\pm 16\text{ sample}$ array lag window.
   - During `init()`, normalized autocorrelation kernels $K_{\text{edge}}[\tau]$ and $K_{\text{diag}}[\tau]$ are precomputed via IFFT of the bandpass weighting tables.
2. **Lag Window Preservation**:
   - The normalized cross-correlation lags for all 6 pairs are copied into a stack buffer `pair_corr[6][45]` (covering lag offsets $[-22, +22]$ samples) consuming only $\sim 1\text{ KB}$ of the 8 KB DSP stack.
3. **Iterative Extraction Loop**:
   - **Pass 0 (Dominant Source)**: Computes the primary sound azimuth via pairwise argmax and $2\times 2$ WLS solver. If valid, the primary source is stored in `MultiDoaResult.sources[0]`. The scaled acoustic imprint of the primary source ($A_p \cdot K_p[\tau - \hat{\tau}_p]$) is subtracted from `pair_corr[p]` for all 6 pairs.
   - **Pass 1..2 (Secondary & Tertiary Sources)**:
     - **Coarse SRP-PHAT Grid Search**: Sweeps 72 candidate azimuths $\theta \in [-180^\circ, +180^\circ)$ with $5^\circ$ resolution:
       $$P(\theta) = \frac{1}{6} \sum_{p=0}^5 \text{pair\_corr}_p\big[\text{round}(\tau_p(\theta))\big]$$
       Algebraically optimized to only 2 multiplications per angle, completing the entire 72-point beamforming search in $< 10\ \mu\text{s}$ CPU time.
     - **Termination Gate**: If $P(\theta_{\text{seed}}) < \text{threshold}$, the iteration stops.
     - **Local Parabolic WLS Refinement**: For each pair, searches within a narrow local window of $\pm 3\text{ samples}$ around $\tau_p(\theta_{\text{seed}})$, fits a 3-point parabolic peak, and solves the $2\times 2$ WLS normal equations with closed-triangle loop-closure validation.
     - **Spatial Separation Constraint**: Requires candidate azimuths to maintain at least $25^\circ$ separation (`DOA_MIN_SOURCE_SEPARATION_DEG`) from all previously accepted sources to avoid reporting residual sidelobes.
     - **CLEAN Subtraction**: The candidate's fitted imprint is subtracted from `pair_corr`, and the loop repeats.

### Adaptive Noise Floor Tracking

The pipeline supports an adaptive noise floor tracker implemented as an asymmetric leaky integrator estimating ambient acoustic noise:

1. **Noise floor tracking**:
   - When the signal level drops below the current noise floor estimate, it tracks downward rapidly ($\alpha_{\text{down}} = 0.02$, $\tau \approx 1\text{ s}$ at $\sim 43\text{ Hz}$).
   - When the signal is between the noise floor and the trigger threshold ($P_{\text{noise}} + \text{SNR}_{\text{margin}}$), it tracks upward slowly ($\alpha_{\text{up}} = 0.005$, $\tau \approx 5\text{ s}$).
   - When an active sound event occurs (signal exceeds trigger threshold), adaptation drops to a very slow rate ($\alpha_{\text{event}} = 0.0005$) to prevent sustained speech or alarms from pulling up the noise floor and extinguishing detection.
   - The noise floor estimate is clamped to $[-85.0, -20.0]\text{ dBFS}$.
2. **Threshold evaluation**:
   - If adaptive thresholding is enabled, the effective threshold is $P_{\text{noise}} + \text{SNR}_{\text{margin}}$ (`noise_floor + snr_margin_db`).
   - If disabled, the fixed threshold `db_threshold` (derived from `sound_level_threshold_db_spl - db_spl_offset`) is used.
   - Signal blocks below the effective threshold bypass the compute-intensive FFT cross-correlation steps.

The sound speed is calculated dynamically from BME280 temperature, relative humidity, and barometric pressure using moist air thermodynamics:

1. Saturation vapor pressure (Magnus-Tetens):
   $$p_{sat} = 610.78 \cdot \exp\left(\frac{17.27 \cdot T_C}{T_C + 237.3}\right) \text{ [Pa]}$$
2. Vapor partial pressure and mole fraction:
   $$p_v = \frac{RH}{100} \cdot p_{sat}, \quad x_w = \frac{p_v}{P}$$
3. Moist air effective molar mass and heat capacity ratio:
   $$M = 0.0289654 \cdot (1 - 0.3780 \cdot x_w) \text{ [kg/mol]}, \quad \gamma = 1.4003 - 0.07 \cdot x_w$$
4. Speed of sound:
   $$c = \sqrt{\frac{\gamma \cdot R \cdot T_K}{M}}$$
   where $R = 8.31446 \text{ J/(mol}\cdot\text{K)}$ and $T_K = T_C + 273.15 \text{ K}$. Default fallbacks are $20.0^\circ\text{C}$, $50.0\%$, and $101325.0\text{ Pa}$ ($c \approx 343.9\text{ m/s}$).

## `DoaResult`

The internal result contains:

| Field | Meaning |
|---|---|
| `angle_deg` | Estimated azimuth in degrees, $[-180^\circ, +180^\circ]$ ($0^\circ = \text{Front/North}$, $+90^\circ = \text{Right/East}$, $\pm 180^\circ = \text{Back/South}$, $-90^\circ = \text{Left/West}$) |
| `delay_us` | Equivalent projected X-axis inter-channel delay in microseconds ($\frac{d \sin\phi}{c} \cdot 10^6$) |
| `confidence` | Weighted mean correlation peak confidence in $[0, 1]$ |
| `rms_db` | Omnidirectional 4-channel digital RMS level in dBFS |
| `valid` | Whether energy, loop-closure consistency, and correlation thresholds were satisfied |

The internal `rms_db` value is not inherently dB SPL. The network and display paths add the configured dB SPL offset when presenting or transmitting it.

## `MultiDoaResult`

When running multi-source detection via `GccPhat::processMulti()`, results are packaged in a `MultiDoaResult` struct:

| Field | Meaning |
|---|---|
| `sources` | Array of `DoaResult` records (up to `DOA_MAX_SOURCES = 3`), sorted by confidence descending |
| `count` | Number of verified simultaneous acoustic sources detected in this frame (`0..3`) |
| `rms_db` | Omnidirectional 4-channel digital RMS level in dBFS |
| `valid` | True if at least one valid sound source was extracted (`count > 0`) |

## Power-On Self-Test (POST)

During startup (`SETUP_BOOT_DELAY_MS = 3000`), the firmware executes an automated hardware verification before starting real-time tasks:

1. **Octal PSRAM**: Verifies that 8 MB OPI PSRAM is detected and available.
2. **BME280 Environmental Sensor**: Probes I2C `0x76`/`0x77` and verifies plausible temperature and pressure readings.
3. **4-Channel I2S Microphones (`evaluate_quad_microphone_health`)**: Captures a synchronized 1024-sample quad block from `I2S0` (`GPIO 6`) and `I2S1` (`GPIO 7`) and verifies:
   - **Flatline Detection**: Rejects any channel (`M0..M3`) where $\text{min} == \text{max}$ or AC $\text{RMS} \le -95\text{ dBFS}$ (disconnected, unpowered, or floating line).
   - **Rail Saturation**: Rejects any channel with total or AC $\text{RMS} \ge -3\text{ dBFS}$ (DC short to rail or severe clipping).
   - **Intra-Bus & Cross-Bus Collision**: Checks pairwise identical sample counts ($< 85\%$) on Bus 0 (`M0` vs `M1`), Bus 1 (`M2` vs `M3`), and across buses (Bus 0 `GPIO 6` vs Bus 1 `GPIO 7`) to detect L/R slot miswiring or shorted data lines.
4. **Dual Reporting & Hardware Degraded Fault Halt**:
   - Outputs a comprehensive ASCII diagnostic table to Serial at 115200 baud.
   - Renders a sequential, staggered color-coded checklist (`PSRAM`, `BME280`, `M0..M3`, `SYNC`) on the ST7735 TFT alongside the countdown progress bar.
   - **Degraded Operation Halt**: If critical hardware fails (PSRAM missing, I2S driver failure, or any microphone flatlined, rail-shorted, or colliding), the firmware halts after the boot countdown, holding a dedicated red `HARDWARE DEGRADED` diagnostic screen on the TFT that specifies the exact failure reason (e.g. `M0 [FAIL] FLATLINE`, `SYNC [FAIL] Bus0 LR`) and instructs the user to fix wiring and reboot. Real-time DSP and network streaming tasks are suspended to prevent corrupted DOA data from reaching the REST backend.
   - Holding GPIO 0 (BOOT button) during both the boot checklist and the degraded halt screen enters Setup Mode (SoftAP captive portal) as usual.

## Dual-core task layout

### Core 1: `DSP_Task`

- Reads synchronized 4-channel I2S DMA data (`read_i2s_quad`).
- Runs 4-channel 6-pair dual-band GCC-PHAT, triangle loop-closure weighting, 2D WLS azimuth estimation, and adaptive noise floor tracking.
- Sends every valid result independently to the display and network queues.
- Never performs Wi-Fi, HTTP, or network batching work.
- Measures processing duration against the current audio-block deadline.
- Resets the task watchdog and periodically reports timing, I2S, queue, and stack statistics.
- Does not perform Wi-Fi, HTTP, display, or BME280 operations; it emits only
  rate-limited diagnostic summaries to Serial.
- Priority: 5.
- Stack: 8192 bytes.

### Core 0: `Sensor_Display`

- Reads the BME280 approximately every two seconds.
- Updates the speed-of-sound calibration.
- Receives results from `display_queue`.
- Applies a circular $S^1$ angle smoothing filter (`DisplaySmoothingFilter`, EMA with $\alpha = 0.25$ and shortest-arc outlier rejection threshold $|\Delta\phi| > 45^\circ$ via `shortestAngularDiff()` and `wrapAngle180()`, confirming source relocation after 2 consecutive consistent readings on $S^1$, and resetting immediately after $>1.5\text{ s}$ of silence) so transitions across $\pm 180^\circ$ (e.g., $+175^\circ \leftrightarrow -175^\circ$) smooth continuously without swinging through $0^\circ$.
- Uploads and network measurements (`network_queue` and API batches) remain strictly raw and physics-consistent (no smoothing is applied to API measurements).
- Prints event information to Serial.
- Draws the full-color $360^\circ$ circular radar UI on the SPI TFT, including an `[AUTO]` pill badge (in cyan) in the top-right corner when adaptive noise floor tracking is enabled (or `[FIX]` in dark gray when disabled).
- Priority: 2.
- Stack: 4096 bytes.

### Core 0: `Network_Task`

- Waits for Wi-Fi.
- Reconnects when disconnected.
- Starts the SNTP client once per Wi-Fi connection instead of restarting it on
  a fixed retry interval. It also accepts
  an optional `server_time_unix_ms` field from the calibration response and
  estimates a server-to-device UTC offset. If both sources are unavailable, it
  transmits monotonic uptime timestamps with an explicit clock flag instead of
  blocking or discarding measurements.
- Refreshes remote calibration at boot and approximately every 60 seconds.
- Receives timestamped results from `network_queue` and batches up to eight
  measurements, flushing a partial batch after 200 ms.
- Sends packed binary batches with HTTP keep-alive and a five-second timeout.
- Targets at most five requests per second.
- Retries each failed batch up to three times with increasing delays.
- Priority: 2.
- Stack: 8192 bytes.

The Arduino `loop()` only delays. Application work is performed by the explicit FreeRTOS tasks.

## Synchronization

- `doa_mutex` protects the `GccPhat` object while DSP processing or calibration updates are running.
- `config_mutex` protects configuration values while they are copied or replaced.
- `display_queue` and `network_queue` contain independent copies of each valid result.
- Temperature, speed of sound, and dB SPL offset are exposed through atomic values where they are shared between tasks.
- Binary batch header clock flags distinguish synchronized UTC timestamps from
  server-synchronized UTC timestamps, and monotonic uptime timestamps.
- Measurement timestamps are based on the monotonic ESP timer; UTC timestamps
  apply a stored offset. This prevents SNTP clock corrections from creating
  backwards batch offsets.

Queue sends are non-blocking. If the bounded network queue is full, the oldest
queued measurement is discarded and the newest result is retained. The display
queue uses the same latest-result policy. Drop counters are included in the
periodic DSP diagnostic summary. The DSP task never waits for the display or
network consumer. The display shows `API OFFLINE` while server delivery is
unavailable, but continues displaying the latest acoustic result.

## Configuration

Configuration is loaded from ESP32 NVS namespace `acoustic`. It contains:

- Wi-Fi SSID and password.
- API base URL and API key.
- Backend-assigned device ID.
- Microphone square side length (`mic_distance`, default 0.082 m).
- Correlation threshold.
- dB SPL threshold.
- dB SPL offset.
- Zero angle rotational offset (`ang_offset`, -180.0 to +180.0 deg).
- Adaptive noise floor tracking state and SNR margin.
- Sample rate and FFT window size.

The sample rate and window size are validated against a whitelist:

| Setting | Supported values |
|---|---|
| Sample rate | 44100 Hz, 48000 Hz |
| FFT/window size | 512, 1024, 2048 samples |

Remote changes are applied by reinitializing the GCC-PHAT buffers and dual I2S
drivers while processing is synchronized. Invalid settings or failed
reconfiguration are rejected and the previous valid configuration is retained.
The active processing deadline is `window_size / sample_rate`.

Every ten seconds, the DSP task emits a rate-limited summary similar to:

```text
DSP stats: blocks=430, deadline_misses=0, max_processing=5250 us,
budget=23219 us, i2s_failures=0, display_drops=0, network_drops=0,
stack_free=6000
```

Task watchdog monitoring is enabled with a thirty-second timeout for the DSP
and network tasks. The timeout accommodates the network task's bounded HTTP
timeouts and retries while still detecting a stuck task; deadline counters separately detect
processing that is too slow even when the task continues to run.

## Display

The 128x160 color SPI TFT (ST7735 / KMR-1.8) shows a full $360^\circ$ circular radar scope (`cx = 64, cy = 44, r = 34`) with the latest valid sound azimuth in $[-180^\circ, +180^\circ]$. Rendering is double-buffered in RAM via a 40 KB `GFXcanvas16` framebuffer to eliminate screen flicker, then transferred over 27 MHz SPI via `drawRGBBitmap`.

- The upper scope (`y = 0..82`) displays outer ($r = 34$) and inner ($r = 17$) concentric range rings, N/E/S/W crosshair axes, and cardinal degree labels (`0`, `+90`, `180`, `-90`).
- The $360^\circ$ polar radar needle is color-coded by calibrated sound pressure level ($> 75\text{ dB SPL}$ red, $> 60\text{ dB SPL}$ yellow, otherwise green).
- The lower telemetry dashboard (`y = 84..159`) displays detected azimuth (`%+6.1f deg`), calibrated volume in dB SPL, moist air sound speed, BME280 temperature, relative humidity, barometric pressure, and live API connectivity status (`API ONLINE` / `API OFFLINE`).
- The display is updated when a valid event arrives from `display_queue` or when backend reachability state changes.
