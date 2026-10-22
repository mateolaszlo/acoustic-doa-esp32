# Acoustic DOA ESP32 Firmware

This directory documents the current ESP32-S3 firmware for a four-microphone planar square array acoustic direction-of-arrival (DOA) locator.

## Documentation

- [Hardware and wiring](doc/hardware.md)
- [Firmware architecture](doc/firmware.md)
- [Build, provisioning, and setup](doc/setup.md)
- [REST API integration](doc/rest-api.md)
- [Testing & verification](doc/testing.md)
- [Troubleshooting](doc/troubleshooting.md)

## Project scope

The firmware:

- Captures four synchronized I2S microphone channels in a planar square array.
- Estimates 6-pair time differences of arrival with dual-band GCC-PHAT.
- Suppresses multipath wall reflections via closed-triangle loop-closure consistency.
- Computes full 360-degree acoustic azimuth (-180 to +180 degrees) via regularized 2D WLS solver.
- Uses local BME280 temperature, relative humidity, and barometric pressure readings for thermodynamic moist-air speed-of-sound compensation.
- Displays a real-time 360-degree radar UI, sound-reactive color needle, and live telemetry on a 1.8" SPI TFT (ST7735 / KMR-1.8 128x160).
- Applies circular S^1 display smoothing across the +/-180 degree back boundary.
- Sends accepted events to a REST API over Wi-Fi.
- Stores device credentials and calibration settings in ESP32 NVS flash.
- Provides a button-triggered setup access point with a DNS-based captive portal.

The backend, database, and web dashboard are separate systems. See [REST API integration](doc/rest-api.md) for the firmware-side contract.

## Testing

The project includes an automated off-target host unit test suite for rapid feature verification and regression prevention without physical ESP32 hardware:

```bash
# Run all 12 unit test suites with AddressSanitizer and UndefinedBehaviorSanitizer
make test

# Or run the shell script
./scripts/run_tests.sh
```

See [Testing documentation](doc/testing.md) for mock details and full suite descriptions.

## Current target

- Board: ESP32-S3-DevKitC-1-N16R8
- Memory: 16 MB flash, 8 MB OPI PSRAM
- USB: two USB ports on the board
- On-board indicator: WS2812 RGB LED (not currently used by the firmware)
- Framework: Arduino for ESP32
- Build system: PlatformIO
- Serial monitor: 115200 baud

## Important current limitations

- The DSP defaults to 44100 Hz with a 1024-sample FFT window. Supported remote values are 44100 or 48000 Hz and 512, 1024, or 2048 samples; other values are rejected.
- The DSP reports raw signal level internally as dBFS. The uploaded `raven_zvoka_db` value is calculated as dBFS plus a user-provided offset and should only be called dB SPL after that offset has been calibrated.
- Measurements are sent in timestamped binary batches of up to eight records, with
  a target limit of five requests per second. Failed batches are retried in RAM
  three times and then dropped; they are not persisted across reboot.
- The calibration response may provide `server_time_unix_ms`; this lets the
  firmware use API-server UTC time even when public NTP is unreachable.
- If neither Internet/NTP nor API-server time is available, measurements
  continue using monotonic uptime timestamps marked in the binary batch header.
- The display continues receiving valid results at DSP rate. When the server is
  unavailable, the network queue keeps newer measurements and the display shows
  `API OFFLINE`.
- GPIO 0 is used for setup detection, but it is also an ESP32-S3 boot strap pin. Holding it low during reset can enter the ROM download bootloader. The setup procedure therefore requires a normal application boot followed by pressing GPIO 0 during the 3-second on-screen startup countdown window.
