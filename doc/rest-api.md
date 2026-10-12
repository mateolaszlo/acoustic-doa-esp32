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
GET /api/v1/naprave/{naprava_id}/kalibracija
x-api-key: <device API key>
Accept: application/json
```

The expected response contains:

```json
{
  "naprava_id": 1,
  "razdalja_mikrofonov_m": 0.08,
  "vzorcevalna_frekvenca": 44100,
  "okno_velikost": 1024,
  "prag_korelacije": 0.40,
  "prag_glasnosti_db": 55.0,
  "db_spl_offset": 96.5,
  "server_time_unix_ms": 1791830000123
}
```

`temperatura_okolice_c` is not required by the current firmware. The local BME280 is the source of temperature for sound-speed compensation.

`db_spl_offset` is optional in the current parser. If absent, the previously stored offset is retained.
`server_time_unix_ms` is optional. When present, the firmware estimates the
server-to-device UTC offset using the midpoint of the calibration request and
uses it for measurement timestamps when NTP has not synchronized the system
clock. Missing or invalid server time does not invalidate otherwise valid
calibration settings; the last valid server offset remains usable until a
new valid synchronization is received.

The firmware validates:

- Positive microphone distance.
- Positive correlation threshold.
- A sound-level threshold above -200 dB.
- Finite floating-point values.
- A supported sample rate: 44100 or 48000 Hz.
- A supported window size: 512, 1024, or 2048 samples.

Invalid responses are rejected and the previous local configuration remains active.
Supported sample-rate/window changes reinitialize the audio and DSP path. If
reinitialization fails, the previous configuration remains active.

## Measurement upload

For valid DSP events, the client sends binary batches:

```http
POST /api/v1/meritve
x-api-key: <device API key>
Content-Type: application/octet-stream
Connection: keep-alive
```

The configured API base URL is still used with `/meritve` appended. The API key
identifies the device; the binary body does not contain a device ID. The
firmware batches up to eight measurements and flushes a partial batch when its
oldest measurement is 200 ms old. Requests are spaced by at least 200 ms, for a
maximum target of five requests per second. The DSP and OLED continue operating
at the active audio-block rate independently of network I/O.

## Binary request body

All integer values are unsigned unless stated otherwise, encoded
little-endian. Floating-point values are IEEE-754 binary32 values. The body
starts with this 12-byte packed header:

| Offset | Size | Field |
|---:|---:|---|
| 0 | 1 | Format version (`1`) |
| 1 | 1 | Measurement count (`1` through `8`) |
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

Each measurement then occupies exactly 20 bytes:

| Offset | Size | Field |
|---:|---:|---|
| 0 | 4 | Millisecond offset from the batch base timestamp |
| 4 | 4 | `angle_deg` as float32 |
| 8 | 4 | `delay_us` as float32 |
| 12 | 4 | Calibrated sound level in dB as float32 |
| 16 | 4 | Correlation confidence as float32 |

Therefore, the body size is `12 + (measurement_count * 20)` bytes. For
example, eight measurements occupy 172 bytes. The timestamp of record `i` is
`base_timestamp_ms + offset_ms`. For monotonic batches, this is uptime relative
to the current boot.

The firmware starts the SNTP client once after Wi-Fi connects, but measurements
are not blocked while synchronization is pending. Measurement timestamps use
the monotonic ESP timer plus the selected UTC offset, so later SNTP clock
corrections cannot make offsets within a batch negative.
The calibration request can independently establish a UTC offset from the
server timestamp.

The previous JSON representation is no longer sent. Its fields correspond to
the binary record as follows:

```json
{
  "kot_stopinje": -34.8,
  "casovna_zakasnitev_us": -132.4,
  "raven_zvoka_db": 76.5,
  "korelacijski_koeficient": 0.78
}
```

`raven_zvoka_db` is calculated as the internal dBFS value plus the configured dB SPL offset. It is only a physically meaningful dB SPL value after calibration.

## Successful responses

Any HTTP status from 200 through 299 is treated as successful. The firmware does not require a response body and does not retrieve or store `dogodek_id`.

The backend may return a body such as:

```json
{
  "status": "success",
  "dogodek_id": 4821,
  "zabelezeno": "2026-10-12T09:15:32.100Z"
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

- Expose `/naprave/:id/kalibracija` with API-key authentication.
- Expose `/meritve` with API-key authentication.
- Accept version 1 of the documented little-endian binary batch format.
- Validate the content length against the count field before decoding.
- Return 2xx for accepted measurements.
- Return useful JSON error bodies for failed requests.
- Not require ambient temperature in the calibration response.
