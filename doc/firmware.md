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

The DSP task reads 1024 stereo samples at 44100 Hz. Each block is processed as follows:

1. Convert the signed I2S samples to floating-point values.
2. Apply a Hann window.
3. Calculate an FFT for each channel.
4. Calculate the cross-power spectrum `X1 * conj(X2)`.
5. Normalize each frequency bin by its magnitude (PHAT weighting).
6. Perform the inverse transform to obtain the correlation function.
7. Search only physically possible lags for the configured microphone distance.
8. Refine the peak with parabolic interpolation.
9. Calculate:
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
- Sends valid results to both result queues.
- Does not perform Wi-Fi, HTTP, OLED, BME280, or serial operations.
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
- Refreshes remote calibration at boot and approximately every 60 seconds.
- Receives results from `network_queue`.
- Performs HTTP requests with a five-second timeout.
- Retries each failed measurement up to three times with increasing delays.
- Priority: 2.
- Stack: 8192 bytes.

The Arduino `loop()` only delays. Application work is performed by the explicit FreeRTOS tasks.

## Synchronization

- `doa_mutex` protects the `GccPhat` object while DSP processing or calibration updates are running.
- `config_mutex` protects configuration values while they are copied or replaced.
- `display_queue` and `network_queue` contain independent copies of each valid result.
- Temperature, speed of sound, and dB SPL offset are exposed through atomic values where they are shared between tasks.

Queue sends are non-blocking. If a queue is full, the event can be lost; the current code does not report individual queue-send failures.

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

The sample rate and window size are validated against the compiled DSP implementation. The current firmware accepts only 44100 Hz and `N_SAMPLES` (1024).

## Display

The OLED shows a semicircular radar with the latest valid angle. The lower line shows angle, local temperature, and calculated sound speed. The display is updated when a valid event arrives rather than at a fixed independent refresh rate.
