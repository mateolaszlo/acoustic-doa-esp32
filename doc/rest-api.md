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
Accept: application/json
```

Measurement uploads additionally send:

```http
Content-Type: application/json
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
  "db_spl_offset": 96.5
}
```

`temperatura_okolice_c` is not required by the current firmware. The local BME280 is the source of temperature for sound-speed compensation.

`db_spl_offset` is optional in the current parser. If absent, the previously stored offset is retained.

The firmware validates:

- Positive microphone distance.
- Positive correlation threshold.
- A sound-level threshold above -200 dB.
- Finite floating-point values.
- A non-zero sample rate and window size.
- Runtime support for exactly 44100 Hz and a 1024-sample window.

Invalid responses are rejected and the previous local configuration remains active.

## Measurement upload

For each valid DSP event, the client sends:

```http
POST /api/v1/meritve
x-api-key: <device API key>
Content-Type: application/json
```

Current request body:

```json
{
  "naprava_id": 1,
  "kot_stopinje": -34.8,
  "casovna_zakasnitev_us": -132.4,
  "raven_zvoka_db": 76.5,
  "korelacijski_koeficient": 0.78
}
```

The `naprava_id` field is sent in addition to the API key. The backend should verify that it matches the device associated with the API key, or explicitly choose the API key as authoritative.

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

1. The event is attempted up to three times.
2. Each retry uses a five-second HTTP timeout.
3. Retry delays increase between attempts.
4. After the final failure, the event is dropped.

The retry queue is RAM-only. Events do not survive reset and are not written to flash.

Examples of backend statuses that should be diagnosed from serial logs:

- `400`: request validation or field mismatch.
- `401`/`403`: invalid or unauthorized API key.
- `404`: wrong base URL, device ID, or route.
- `5xx`: backend/server failure.
- Negative client status: connection, DNS, timeout, or transport failure.

## Backend compatibility checklist

The backend should:

- Expose `/naprave/:id/kalibracija` with API-key authentication.
- Expose `/meritve` with API-key authentication.
- Accept the exact Slovenian JSON field names shown above.
- Accept the extra `naprava_id` field or intentionally ignore it after validating the API-key mapping.
- Return 2xx for accepted measurements.
- Return useful JSON error bodies for failed requests.
- Not require ambient temperature in the calibration response.
