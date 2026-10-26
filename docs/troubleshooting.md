# Troubleshooting

## Build problems

Run the build from the directory containing `platformio.ini`:

```powershell
pio run
```

If PlatformIO is not found, use the PlatformIO environment installed by VS Code or install PlatformIO Core. The project uses the project-local `esp32-s3-devkitc-1-n16r8` board definition.

## Device enters the ROM bootloader

GPIO 0 was held low during reset or power-on. Release GPIO 0 and reset normally. To enter firmware setup, boot normally first and press GPIO 0 during the startup detection delay.

## Setup AP is not visible

Check the serial monitor at 115200 baud. The firmware should print:

```text
Setup AP active at 192.168.4.1
```

If it does not:

- Confirm GPIO 0 was pressed during the application startup window.
- Confirm the button connects GPIO 0 to GND.
- Confirm the board was not forced into the ROM bootloader.
- Manually open `http://192.168.4.1` after connecting to the AP.

## Wi-Fi does not connect after provisioning

Check the serial monitor (115200 baud). The firmware provides automatic diagnostics for connection attempts and disconnections:

- **Wrong SSID or router out of range**:
  ```text
  [WiFi] Disconnected: SSID not found (incorrect SSID name or out of range) (reason 201: NO_AP_FOUND). Check SSID name or router distance!
  ```
- **Wrong Wi-Fi Password**:
  ```text
  [WiFi] Disconnected: Authentication failed (incorrect Wi-Fi password) (reason 202: AUTH_FAIL). Check Wi-Fi password in setup!
  ```
  *(Also reported as `reason 15: 4WAY_HANDSHAKE_TIMEOUT` or `reason 204: HANDSHAKE_TIMEOUT`)*
- **Lost connection / beacon timeout**:
  ```text
  [WiFi] Disconnected: Beacon timeout (connection lost / signal too weak) (reason 200: BEACON_TIMEOUT)
  ```
- **Anti-spam throttling**: During continuous auto-reconnection loops, repeated identical error messages are suppressed and summarized every 15 seconds:
  ```text
  [WiFi] Still disconnected: SSID not found (incorrect SSID name or out of range) (reason 201: NO_AP_FOUND, 12 attempts)
  ```
- **Successful connection**:
  ```text
  [WiFi] Connected to 'MyNetwork'! IP: 192.168.1.150, RSSI: -54 dBm, Channel: 6
  ```

If credentials need updating, power on the device while holding GPIO 0 during the boot window to re-enter setup mode (`192.168.4.1`).

## SPI TFT Display (KMR-1.8 / ST7735) Issues

### Display is completely blank / white
- Confirm SPI wiring: CS=GPIO 10, MOSI/SDA=GPIO 11, SCLK/SCK=GPIO 12, DC/A0=GPIO 13, RST=GPIO 14.
- Confirm LED pin is connected to 3.3V (backlight power).
- **Power supply & Jumper J1**: If powering VCC with 3.3V, check the `J1` zero-ohm jumper on the back of the KMR-1.8 board. Unbridged J1 drops voltage through the onboard LDO; bridge J1 with solder for 3.3V input, or power VCC from the 5V rail (VIN).

### Inverted colors (e.g., white background instead of black)
- Different ST7735 revisions use different color inversion standards. In `src/config/app_config.h`, change `constexpr bool TFT_INVERT_COLORS = false;` to `true`.
- If a 1–2 pixel border or static noise is visible at screen edges, change `display.initR(INITR_BLACKTAB)` in `RadarDisplay::init()` to `INITR_GREENTAB` or `INITR_18BLACKTAB`.

## BME280 is not detected

The firmware checks `0x76` and then `0x77`. Confirm:

- SDA/SCL wiring.
- Power and ground.
- The breakout is actually a BME280 rather than a similar BMP280 module.
- The module address selector.

If unavailable, the firmware retains its default temperature/sound-speed values but cannot perform live environmental compensation.

## No valid DOA events

Check:

- Microphones share BCLK (GPIO 5) and WS (GPIO 4).
- Microphones are set to their designated L/R slots (GND for Left, VDD for Right).
- BCLK (GPIO 5), WS (GPIO 4), and DATA lines (GPIO 6 for Bus 0, GPIO 7 for Bus 1) are wired correctly.
- Microphone distance matches the configured physical spacing.
- The signal is above the configured threshold.
- The correlation threshold is not too high.
- Microphones are mechanically fixed and have a common ground.

The firmware ignores incomplete I2S reads and processes only complete blocks.
The firmware operates on fixed 1024-sample blocks (`AUDIO_FFT_WINDOW_SIZE = 1024`)
at 44100 or 48000 Hz.

## Radar needle is biased toward one side (e.g. +7° offset)

If forward sounds point to a non-zero angle (e.g. `+7.0°`) and the needle reaches `+90°` much more easily than `-90°`:

- **Cause**: Dual digital I2S microphones transmit in alternating WS slots (Left on WS low, Right on WS high). Combined with individual MEMS decimation filter group delays and breadboard depth tolerances, this introduces a static inter-channel timing skew (e.g. ~1 sample $\approx 22.7\ \mu\text{s} \equiv 5.5^\circ\text{ to }7^\circ$ at 82 mm).
- **Solution**:
  1. Measure the exact acoustic port hole-to-hole distance (default is `0.082 m` / 82 mm).
  2. Hold the BOOT button on startup to open the Captive Portal (`192.168.4.1`).
  3. Enter the observed forward angle into `Zero angle offset (deg)` (e.g. `7.0`).
  4. Save and reboot. The firmware applies the angular offset trim on the circular manifold $\phi_{\text{cal}} = \text{wrap}_{180}(\phi_{\text{raw}} - \phi_{\text{offset}})$ (where $\phi_{\text{offset}}$ is `angle_offset_deg`), restoring accurate, symmetric full $360^\circ$ $[-180^\circ, +180^\circ]$ tracking.

## API returns 401 or 403 (Invalid API Key)

The firmware automatically detects unauthorized or rejected API keys on both calibration (`GET /devices/{id}/calibration`) and measurement streaming (`POST /measurements`), printing:
```text
[API] CRITICAL: Measurement batch rejected (HTTP 401)! Invalid or unauthorized API key.
[API] CRITICAL: Calibration rejected (HTTP 403)! Invalid or unauthorized API key for device ID 1.
```

To resolve:
- Confirm the API key in NVS matches the backend configuration.
- Confirm the backend accepts the `x-api-key` HTTP header.
- Re-enter the API key through setup mode (`192.168.4.1`) by holding GPIO 0 on boot.
- Repetitive 401/403 errors are automatically throttled to 1 log every 15 seconds to prevent serial flooding.

## API returns 400

Compare the backend validation with [REST API integration](rest-api.md). The
measurement body is a versioned little-endian binary batch with a 12-byte
header and one 20-byte record per measurement. It uses:

- Unix UTC milliseconds in the batch header.
- A per-record millisecond offset.
- `angle_deg`, `delay_us`, calibrated dB, and confidence as float32 values.

The endpoint must accept `Content-Type: application/octet-stream` and validate
the body length against the record count.

## Calibration is rejected

The response must contain valid numeric values for distance, correlation threshold, sound threshold, sample rate, and window size. The firmware accepts sample rates of 44100 or 48000 Hz and requires a window size of 1024 (other window sizes are rejected).

Ambient temperature is intentionally not required in the response because the local BME280 supplies it.

## Measurements are dropped

The client retries a batch three times in RAM. If all attempts fail, the
serial log reports that the batch was dropped. During an outage, the bounded
network queue discards its oldest measurements when full and retains newer
ones. Failed measurements are not persisted across reboot. The display shows
`API OFFLINE` while delivery is unavailable.

The two event queues have finite capacity. The display keeps the newest result
when full. Network batches contain up to 24 records and are spaced by at
least 200 ms. The network queue keeps newer pending measurements when full;
older pending measurements may be discarded during prolonged outages by
design. The retry queue is not an archive of every DSP block.
