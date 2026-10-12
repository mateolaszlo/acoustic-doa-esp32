# Acoustic DOA ESP32 Firmware

This directory documents the current ESP32-S3 firmware for a two-microphone acoustic direction-of-arrival (DOA) locator.

## Documentation

- [Hardware and wiring](doc/hardware.md)
- [Firmware architecture](doc/firmware.md)
- [Build, provisioning, and setup](doc/setup.md)
- [REST API integration](doc/rest-api.md)
- [Troubleshooting](doc/troubleshooting.md)

## Project scope

The firmware:

- Captures two synchronized I2S microphone channels.
- Estimates time difference of arrival with GCC-PHAT.
- Converts the delay into an angle from -90 to +90 degrees.
- Uses a local BME280 temperature reading for speed-of-sound compensation.
- Displays the latest event on an SSD1306 OLED.
- Sends accepted events to a REST API over Wi-Fi.
- Stores device credentials and calibration settings in ESP32 NVS flash.
- Provides a button-triggered setup access point with a DNS-based captive portal.

The backend, database, and web dashboard are separate systems. See [REST API integration](doc/rest-api.md) for the firmware-side contract.

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
- Failed measurements are retried in RAM three times and then dropped. They are not persisted across reboot.
- GPIO 0 is used for setup detection, but it is also an ESP32-S3 boot strap pin. Holding it low during reset can enter the ROM download bootloader. The setup procedure therefore requires a normal application boot followed by pressing GPIO 0 during the startup detection window.
