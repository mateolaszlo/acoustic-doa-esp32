# Build, provisioning, and setup

## Requirements

- Windows, macOS, or Linux with PlatformIO.
- ESP32-S3-DevKitC-1-N16R8 (16 MB flash, 8 MB OPI PSRAM).
- USB data cable.
- Two compatible I2S microphones.
- 1.8" SPI TFT display (ST7735 / KMR-1.8 128x160).
- BME280 I2C sensor.
- A backend reachable from the ESP32 over the configured network.

## PlatformIO configuration

The project is configured in [platformio.ini](../platformio.ini):

- Environment: `esp32-s3-devkitc-1` using the project-local N16R8 board definition.
- Platform: Espressif 32
- Framework: Arduino
- Serial monitor speed: 115200
- Libraries: ArduinoJson, Adafruit GFX, Adafruit ST7735 and ST7789 Library, Adafruit BME280

Build from the project root:

```powershell
pio run
```

Upload:

```powershell
pio run --target upload
```

Open the serial monitor:

```powershell
pio device monitor
```

The build output directory `.pio/` should not be committed.

## First boot provisioning

The real Wi-Fi credentials, API URL, and API key are not compiled into the firmware. Empty compile-time defaults are used. Values are stored in NVS after setup.

### GPIO 0 warning

GPIO 0 is also an ESP32-S3 boot strap pin. Do **not** hold GPIO 0 low while pressing reset or powering on; that can select the ROM download bootloader and prevent `setup()` from running.

With the current firmware:

1. Boot or reset the board normally with GPIO 0 (BOOT button) released.
2. The TFT display turns on immediately and shows a 3-second countdown with the message: **"Press BOOT button to enter Setup"**.
3. Press and hold the BOOT button (GPIO 0) during this window.
4. The display switches to **SETUP MODE**, showing the Wi-Fi AP name (`Acoustic-Locator-Setup`), password (`configure123`), and portal URL (`http://192.168.4.1`).
5. Connect your phone or computer to the AP and visit `http://192.168.4.1`.
6. Enter the required values and submit the form.
7. The firmware writes values to NVS and restarts.

The setup form contains:

- Wi-Fi SSID.
- Wi-Fi password. Leave blank to retain the stored password.
- API URL.
- API key. Input type password; leave blank to retain the stored key.
- Backend device ID.
- Microphone distance in meters.
- dB SPL offset.
- Fixed sound level threshold (dB SPL).
- Enable adaptive noise floor threshold (checkbox). When enabled, sound triggers when rising above the ambient noise floor by the configured SNR margin instead of relying solely on a fixed threshold.
- SNR margin (dB). Configurable SNR margin above ambient noise floor (default 8.0 dB).

The setup AP is not started during normal boots. It is entered only when the setup input is detected during the startup check.

## Persistent configuration

NVS values survive reset and firmware restart. Reflashing the application normally does not erase NVS, although an erase-flash operation or explicit NVS erase will remove them.

The last valid remote calibration is retained locally. If the backend is unavailable, the device continues using that configuration.

## dB SPL offset

The DSP produces dBFS. The setup value is an offset applied as:

```text
dB SPL = dBFS + configured offset
```

This offset must be measured for the exact microphone, gain, and hardware arrangement. A value copied from another device is not necessarily valid.

## Reconfiguration

To change credentials or device settings:

1. Boot the device normally.
2. Press GPIO 0 during the startup detection window.
3. Connect to the setup AP.
4. Submit the new values.

The new configuration is saved and the device restarts.
