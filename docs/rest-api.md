# REST API integration

This document describes the HTTP behavior implemented by the ESP32 client. The backend must expose compatible routes and validation rules.

## Base URL

The configured API base URL is stored in NVS and should include the version prefix, for example:

```text
http://192.168.1.100:3000/api/v1
```

Do not use `localhost` in the ESP32 configuration. From the ESP32, `localhost` means the ESP32 itself.

## Authentication

Every device request uses:

```http
x-api-key: <device API key>
```

Calibration requests additionally send:

```http
Accept: application/json
```

Measurement uploads additionally send:

```http
Content-Type: application/octet-stream
Connection: keep-alive
```

The API key is entered through provisioning and stored in NVS. It is not retrieved from a user JWT.

## Calibration request

The network task requests calibration at startup after Wi-Fi connects and approximately every 60 seconds:

```http
GET /api/v1/devices/{device_id}/calibration
x-api-key: <device API key>
Accept: application/json
```

The expected response contains:

```json
{
  "device_id": 1,
  "microphone_distance_m": 0.082,
  "sample_rate": 44100,
  "window_size": 1024,
  "correlation_threshold": 0.40,
  "sound_level_threshold_db": 55.0,
  "db_spl_offset": 96.5,
  "angle_offset_deg": 0.0,
  "adaptive_threshold_enabled": true,
  "snr_margin_db": 8.0,
  "sector_width_deg": 15.0,
  "sector_hysteresis_deg": 3.0,
  "tracker_hold_timeout_ms": 2500,
  "server_time_unix_ms": 1791830000123
}
```

`ambient_temperature_c` is not required by the current firmware. The local BME280 is the source of temperature for sound-speed compensation.

`angle_offset_deg` (float in degrees, between `-180.0` and `+180.0`) is optional. Specifies the rotational zero-angle calibration trim in degrees to compensate for hardware mounting orientation or array alignment offsets, wrapped circularly into $[-180^\circ, +180^\circ]$.
`db_spl_offset` is optional in the current parser. If absent, the previously stored offset is retained.
`adaptive_threshold_enabled` (boolean) is optional. When true, enables adaptive noise floor tracking; when false, fixed thresholding via `sound_level_threshold_db` is enforced. If absent, the previous setting is retained.
`snr_margin_db` (float in dB) is optional. Specifies the minimum SNR margin above the estimated ambient noise floor required to trigger DOA calculation. If absent, the previous margin is retained.
`sector_width_deg` (float in degrees, between `5.0` and `90.0`, default `15.0`) is optional. Configures the discrete angular sector width for the 2-speaker sector tracker (e.g. `15.0` = 24 sectors, `10.0` = 36 sectors).
`sector_hysteresis_deg` (float in degrees, between `0.0` and `15.0` and $\le \text{sector\_width\_deg}$, default `3.0`) is optional. Configures the Schmitt-trigger deadband margin beyond each sector boundary.
`tracker_hold_timeout_ms` (unsigned integer in milliseconds, between `200` and `30000`, default `2500`) is optional. Configures the conversational speech-pause hold duration before an inactive speaker slot emits a `LEAVE` event.
`server_time_unix_ms` is optional. When present, the firmware estimates the
server-to-device UTC offset using the midpoint of the calibration request and
uses it for measurement timestamps when NTP has not synchronized the system
clock. Missing or invalid server time does not invalidate otherwise valid
calibration settings; the last valid server offset remains usable until a
new valid synchronization is received.

The firmware validates:

- Positive microphone distance (square array side length $d > 0$).
- Positive correlation threshold.
- A sound-level threshold above -200 dB.
- Finite floating-point values.
- A supported sample rate: 44100 or 48000 Hz.
- A supported window size: 1024 samples (fixed; other sizes are rejected).
- Non-negative, finite SNR margin ($0.0 \le \text{SNR}_{\text{margin}} \le 60.0\text{ dB}$, via `snr_margin_db`).
- Finite zero-angle offset within full-circle bounds ($-180.0^\circ \le \phi_{\text{offset}} \le +180.0^\circ$, via `angle_offset_deg`).

Invalid responses are rejected and the previous local configuration remains active.
Supported sample-rate changes update the audio and DSP path in-place. If
reinitialization fails, the previous configuration remains active.

## Measurement upload

For confirmed speaker sector transitions (`ENTER`, `MOVE`, `HEARTBEAT`, `LEAVE`), the client sends binary batches:

```http
POST /api/v1/measurements
x-api-key: <device API key>
Content-Type: application/octet-stream
Connection: keep-alive
```

The configured API base URL is still used with `/measurements` appended. The API key
identifies the device; the binary body does not contain a device ID. The
firmware batches up to 24 state-transition records and flushes a partial batch when its
oldest record is 30 ms old (`NETWORK_BATCH_FLUSH_INTERVAL_MS = 30`). Requests are spaced by at least 200 ms, for a
maximum target of five requests per second. The DSP and local TFT radar display continue operating
at the active audio-block rate (~43.1 Hz) independently of network I/O.

## Binary request body

All integer values are unsigned unless stated otherwise, encoded
little-endian. Floating-point values are IEEE-754 binary32 values. The body
starts with this 12-byte packed header:

| Offset | Size | Field |
|---:|---:|---|
| 0 | 1 | Format version (`1`) |
| 1 | 1 | Measurement count (`1` through `24`) |
| 2 | 2 | Clock flags |
| 4 | 8 | Base timestamp in milliseconds |

Clock flags are:

| Value | Meaning |
|---:|---|
| `0x0001` | Base timestamp is synchronized Unix UTC time |
| `0x0003` | Base timestamp is Unix UTC time synchronized from the API server |
| `0x0004` | Base timestamp is monotonic device uptime; it is not calendar time |

The firmware uses UTC timestamps after NTP synchronization. If NTP is
unavailable but the calibration response includes `server_time_unix_ms`, it
uses the API server as the UTC source and marks batches `0x0003`. If neither
source is available, it continues collecting and sending measurements with
monotonic uptime timestamps marked `0x0004`. A batch never mixes clock
sources.

Each measurement record then occupies exactly 20 bytes:

| Offset | Size | Field |
|---:|---:|---|
| 0 | 4 | Millisecond offset from the batch base timestamp (`uint32_t`) |
| 4 | 4 | `angle_deg` as float32 (quantized sector center azimuth in $[-180^\circ, +180^\circ]$: $0^\circ = \text{Front/North}$, $+90^\circ = \text{Right/East}$, $\pm 180^\circ = \text{Back/South}$, $-90^\circ = \text{Left/West}$) |
| 8 | 1 | `slot_id` (`uint8_t`: persistent tracked speaker slot `0` or `1`) |
| 9 | 1 | `sector_id` (`uint8_t`: discrete sector index `0 .. 23` for $15^\circ$ sectors, or `0xFF` on `LEAVE`) |
| 10 | 1 | `event_type` (`uint8_t`: `1 = ENTER`, `2 = MOVE`, `3 = HEARTBEAT`, `4 = LEAVE`) |
| 11 | 1 | `active_sources` (`uint8_t`: total currently active tracked speakers `0 .. 2` after this transition) |
| 12 | 4 | Calibrated sound level in dB as float32 (`rms_db + db_spl_offset`) |
| 16 | 4 | Correlation confidence as float32 (smoothed tracker confidence in $[0, 1]$) |

Each batch terminates with a 4-byte CRC32 trailer appended immediately after the last measurement record (at offset $12 + \text{count} \times 20$):

| Offset | Size | Field |
|---:|---:|---|
| $12 + (N \times 20)$ | 4 | IEEE 802.3 CRC32 trailer (little-endian unsigned 32-bit integer) |

Therefore, the total body size formula is:
$$\text{Total Size} = 12 + (\text{measurement\_count} \times 20) + 4\text{ bytes}$$

For example:
- A 1-measurement batch occupies **36 bytes**.
- An 8-measurement batch occupies **176 bytes**.
- A full batch of 24 measurements occupies **496 bytes**.

The CRC32 is calculated over all preceding bytes in the payload (the entire 12-byte header plus all $N \times 20$ bytes of measurement records) using standard IEEE 802.3 CRC-32 (polynomial `0x04C11DB7`, reflected `0xEDB88320`, initial `0xFFFFFFFF`, final XOR `0xFFFFFFFF`). On the ESP32-S3 hardware, this is computed at high speed with zero flash overhead using the on-chip ROM function `esp_rom_crc32_le`.

The timestamp of record `i` is
`base_timestamp_ms + offset_ms`. For monotonic batches, this is uptime relative
to the current boot.

The firmware starts the SNTP client once after Wi-Fi connects, and periodically
disciplines `ntp_utc_offset_ms` every 10 minutes from `gettimeofday()`. Measurement
timestamps use the monotonic ESP timer plus the selected UTC offset, so gradual drift
discipline cannot make offsets within a batch negative.
When SNTP is unavailable, periodic 60-second calibration requests independently establish
and discipline a fallback Server UTC offset using RTT gating and EMA low-pass filtering.

The previous JSON representation is no longer sent. Its fields correspond to
the binary record as follows:

```json
{
  "angle_deg": -34.8,
  "delay_us": -132.4,
  "sound_level_db": 76.5,
  "confidence": 0.78
}
```

`sound_level_db` is calculated as the internal dBFS value plus the configured dB SPL offset. It is only a physically meaningful dB SPL value after calibration.

## Successful responses

Any HTTP status from 200 through 299 is treated as successful. The firmware does not require a response body and does not retrieve or store `event_id`.

The backend may return a body such as:

```json
{
  "status": "success",
  "event_id": 4821,
  "recorded_at": "2026-10-12T09:15:32.100Z"
}
```

The body is logged only for failures; success is determined by the 2xx status.

## Failure handling

For calibration failures, the client logs the HTTP status and response body and retains the last valid configuration.

For measurement failures:

1. The batch is attempted up to three times.
2. Each retry uses a five-second HTTP timeout.
3. Retry delays increase between attempts.
4. After the final failure, the batch is dropped from RAM.

The measurement queue is RAM-only. While Wi-Fi/API delivery is unavailable,
the DSP and display continue operating. If the bounded network queue fills,
the oldest queued measurements are discarded and the newest measurements are
retained. Measurements do not survive reset and are not written to flash.

Examples of backend statuses that should be diagnosed from serial logs:

- `400`: request validation or field mismatch.
- `401`/`403`: invalid or unauthorized API key.
- `404`: wrong base URL or route.
- `5xx`: backend/server failure.
- Negative client status: connection, DNS, timeout, or transport failure.

## Backend compatibility checklist

The backend should:

- Expose `/devices/:id/calibration` with API-key authentication.
- Expose `/measurements` with API-key authentication.
- Accept version 1 of the documented little-endian binary batch format with measurement counts up to 24 (payload up to 496 bytes, including the 4-byte CRC32 trailer; ensure backend validator does not reject count > 8 or payloads up to 496 bytes).
- Validate the content length ($12 + \text{count} \times 20 + 4$ bytes) against the count field before decoding.
- Verify the 4-byte IEEE 802.3 CRC32 trailer over the first $12 + \text{count} \times 20$ bytes of the payload.
- Return 2xx for accepted measurements.
- Return useful JSON error bodies for failed requests.
- Not require ambient temperature in the calibration response.
