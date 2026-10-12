# Firmware architecture

## Source files

| File | Responsibility |
|---|---|
| `src/main.cpp` | Setup portal, NVS configuration, Wi-Fi/API, OLED/BME280, FreeRTOS tasks |
| `src/i2s_sampler.cpp` | I2S driver installation and stereo DMA reads |
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

The sound speed is calculated from the BME280 temperature using:

```text
c = 331.3 * sqrt(1 + temperature_C / 273.15)
```

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

## Dual-core task layout

### Core 1: `DSP_Task`

- Reads I2S DMA data.
- Runs GCC-PHAT.
- Sends every valid result independently to the display and network queues.
- Never performs Wi-Fi, HTTP, or network batching work.
- Measures processing duration against the current audio-block deadline.
- Resets the task watchdog and periodically reports timing, I2S, queue, and stack statistics.
- Does not perform Wi-Fi, HTTP, OLED, or BME280 operations; it emits only
  rate-limited diagnostic summaries to Serial.
- Priority: 5.
- Stack: 8192 bytes.

### Core 0: `Sensor_Display`

- Reads the BME280 approximately every two seconds.
- Updates the speed-of-sound calibration.
- Receives results from `display_queue`.
- Prints event information to Serial.
- Draws the OLED radar.
- Priority: 2.
- Stack: 4096 bytes.

### Core 0: `Network_Task`

- Waits for Wi-Fi.
- Reconnects when disconnected.
- Attempts UTC synchronization with NTP after Wi-Fi connects. It also accepts
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

Queue sends are non-blocking. If the bounded network queue is full, the oldest
queued measurement is discarded and the newest result is retained. The display
queue uses the same latest-result policy. Drop counters are included in the
periodic DSP diagnostic summary. The DSP task never waits for the display or
network consumer. The OLED shows `API OFFLINE` while server delivery is
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

The OLED shows a semicircular radar with the latest valid angle. The lower line shows angle, local temperature, and calculated sound speed. The display is updated when a valid event arrives rather than at a fixed independent refresh rate.
