# Firmware architecture

## Source files

| File | Responsibility |
|---|---|
| `src/main.cpp` | Setup portal, NVS configuration, Wi-Fi/API, TFT/BME280, FreeRTOS tasks |
| `src/i2s_sampler.cpp` | I2S driver installation and stereo DMA reads |
| `src/display_filter.h` | Radar visual angle smoothing filter (EMA with outlier rejection) |
| `src/post_checker.h` | Power-On Self-Test (POST) statistical heuristics for I2S microphones |
| `src/gcc_phat.h` | `DoaResult`, `GccPhat`, and FFT-size declarations |
| `src/gcc_phat.cpp` | Windowing, FFT, GCC-PHAT, peak search, interpolation, and angle calculation |
| `platformio.ini` | Board, framework, monitor, and library dependencies |

## Audio processing

The DSP task reads a configured stereo block. The default is 1024 samples at
44100 Hz. Supported sample rates are 44100 and 48000 Hz; supported FFT/window
sizes are 512, 1024, and 2048 samples. Each block is processed as follows:

1. Convert the signed I2S samples to floating-point values.
2. Apply a Hann window.
3. Calculate an FFT for each channel.
4. Calculate the cross-power spectrum `X1 * conj(X2)`.
5. Normalize each frequency bin by its magnitude (PHAT weighting).
6. Apply a mirrored 300-2200 Hz bandpass with 100 Hz raised-cosine
   transition ramps to suppress spatial aliasing and abrupt spectral edges.
7. Perform the inverse transform to obtain the correlation function.
8. Search only physically possible lags for the configured microphone distance.
9. Refine the peak with parabolic interpolation.
10. Calculate:
   - Delay in microseconds.
   - Angle using `asin(c * delay / distance)`.
   - Correlation confidence.
   - Raw level in dBFS.

### Adaptive Noise Floor Tracking

The pipeline supports an adaptive noise floor tracker implemented as an asymmetric leaky integrator estimating ambient acoustic noise:

1. **Noise floor tracking**:
   - When the signal level drops below the current noise floor estimate, it tracks downward rapidly ($\alpha_{\text{down}} = 0.02$, $\tau \approx 1\text{ s}$ at $\sim 43\text{ Hz}$).
   - When the signal is between the noise floor and the trigger threshold ($\text{noise\_floor} + \text{snr\_margin}$), it tracks upward slowly ($\alpha_{\text{up}} = 0.005$, $\tau \approx 5\text{ s}$).
   - When an active sound event occurs (signal exceeds trigger threshold), adaptation drops to a very slow rate ($\alpha_{\text{event}} = 0.0005$) to prevent sustained speech or alarms from pulling up the noise floor and extinguishing detection.
   - The noise floor estimate is clamped to $[-95.0, -5.0]\text{ dBFS}$.
2. **Threshold evaluation**:
   - If adaptive thresholding is enabled, the effective threshold is $\text{noise\_floor} + \text{snr\_margin\_db}$.
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
| `angle_deg` | Estimated angle, nominally -90 to +90 degrees |
| `delay_us` | Estimated inter-channel delay |
| `confidence` | Correlation peak used for validity |
| `rms_db` | Raw digital RMS level in dBFS |
| `valid` | Whether the correlation threshold was exceeded |

The internal `rms_db` value is not inherently dB SPL. The network and display paths add the configured dB SPL offset when presenting or transmitting it.

## Power-On Self-Test (POST)

During startup (`SETUP_BOOT_DELAY_MS = 3000`), the firmware executes an automated hardware verification before starting real-time tasks:

1. **Octal PSRAM**: Verifies that 8 MB OPI PSRAM is detected and available.
2. **BME280 Environmental Sensor**: Probes I2C `0x76`/`0x77` and verifies plausible temperature and pressure readings.
3. **I2S Microphones (Statistical Heuristics)**: Captures a 1024-sample block and verifies:
   - **Flatline Detection**: Rejects channels where $\text{min} == \text{max}$ or $\text{RMS} \le -95\text{ dBFS}$ (disconnected, unpowered, or floating line).
   - **Rail Saturation**: Rejects channels with $\text{RMS} \ge -3\text{ dBFS}$ (DC short to rail).
   - **Stereo Collision**: Counts pairwise identical samples across the 1024-sample block to ensure both microphones are not driving the same channel or shorted together.
4. **Dual Reporting**:
   - Outputs a comprehensive ASCII diagnostic table to Serial at 115200 baud.
   - Renders a color-coded checklist (`[OK]` in green, `[FAIL]` in red) on the ST7735 TFT alongside the countdown progress bar.
   - Holding GPIO 0 during this screen still enters Setup Mode as usual.

## Dual-core task layout

### Core 1: `DSP_Task`

- Reads I2S DMA data.
- Runs GCC-PHAT and adaptive noise floor estimation.
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
- Applies an angle smoothing filter (Exponential Moving Average with $\alpha = 0.25$ and outlier rejection threshold $\Delta\theta > 45^\circ$, confirming source relocation after 2 consecutive jumps, and resetting immediately after $>1.5\text{ s}$ of silence) specifically for rendering the visual radar needle on the SPI TFT without needle jitter.
- Uploads and network measurements (`network_queue` and API batches) remain strictly raw and physics-consistent (no smoothing is applied to API measurements).
- Prints event information to Serial.
- Draws the full-color radar UI on the SPI TFT, including an `[AUTO]` pill badge (in cyan) in the top-right corner when adaptive noise floor tracking is enabled (or `[FIX]` in dark gray when disabled).
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
- Microphone distance.
- Correlation threshold.
- dB SPL threshold.
- dB SPL offset.
- Sample rate and FFT window size.

The sample rate and window size are validated against a whitelist:

| Setting | Supported values |
|---|---|
| Sample rate | 44100 Hz, 48000 Hz |
| FFT/window size | 512, 1024, 2048 samples |

Remote changes are applied by reinitializing the GCC-PHAT buffers and I2S
driver while processing is synchronized. Invalid settings or failed
reconfiguration are rejected and the previous valid configuration is retained.
The active processing deadline is `window_size / sample_rate`.

Every ten seconds, the DSP task emits a rate-limited summary similar to:

```text
DSP stats: blocks=430, deadline_misses=0, max_processing=5100 us,
budget=23219 us, i2s_failures=0, display_drops=0, network_drops=0,
stack_free=6000
```

Task watchdog monitoring is enabled with a thirty-second timeout for the DSP
and network tasks. The timeout accommodates the network task's bounded HTTP
timeouts and retries while still detecting a stuck task; deadline counters separately detect
processing that is too slow even when the task continues to run.

## Display
 
The 128x160 color SPI TFT (ST7735 / KMR-1.8) shows a semicircular radar with the latest valid sound angle. Rendering is double-buffered in RAM via a 40 KB `GFXcanvas16` framebuffer to eliminate screen flicker, then transferred over 27 MHz SPI via `drawRGBBitmap`.
 
- The upper panel displays the semicircular angle grid with angle labels (-90°, 0°, +90°).
- The dynamic radar needle is color-coded by calibrated sound pressure level ($> 75\text{ dB SPL}$ red, $> 60\text{ dB SPL}$ yellow, otherwise green).
- The lower telemetry panel displays detected angle, calibrated volume in dB SPL, BME280 temperature, calculated moist air sound speed, and live API connectivity status (`API ONLINE` / `API OFFLINE`).
- The display is updated when a valid event arrives from `display_queue` or when backend reachability state changes.
