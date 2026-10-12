# Troubleshooting

## Build problems

Run the build from the directory containing `platformio.ini`:

```powershell
pio run
```

If PlatformIO is not found, use the PlatformIO environment installed by VS Code or install PlatformIO Core. The project has previously built successfully for `esp32-s3-devkitc-1`.

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

Check:

- SSID and password spelling.
- The device is within range.
- The configured network is compatible with the board and access point.
- The API URL is a LAN-reachable address, not `localhost`.
- The stored configuration was accepted and the board restarted.

The device retries Wi-Fi connection in the network task and logs HTTP failures, but it does not show a full Wi-Fi diagnostic page.

## OLED is blank

- Confirm SDA is GPIO 8 and SCL is GPIO 9.
- Confirm the OLED is powered and grounded.
- Confirm the address is `0x3C`.
- Check serial output for `SSD1306 ni najden!`.

## BME280 is not detected

The firmware checks `0x76` and then `0x77`. Confirm:

- SDA/SCL wiring.
- Power and ground.
- The breakout is actually a BME280 rather than a similar BMP280 module.
- The module address selector.

If unavailable, the firmware retains its default temperature/sound-speed values but cannot perform live environmental compensation.

## No valid DOA events

Check:

- Both microphones share BCLK, WS, and DATA.
- One microphone is in the left slot and the other in the right slot.
- GPIO 4, 5, and 6 are wired correctly.
- Microphone distance matches the configured physical spacing.
- The signal is above the configured threshold.
- The correlation threshold is not too high.
- Microphones are mechanically fixed and have a common ground.

The firmware ignores incomplete I2S reads and processes only complete 1024-sample stereo blocks.

## API returns 401 or 403

- Confirm the API key in NVS.
- Confirm the backend maps that key to the configured device.
- Confirm the `x-api-key` header is accepted.
- Re-enter the key through setup mode if necessary.

## API returns 400

Compare the backend validation with [REST API integration](rest-api.md). The current measurement body includes:

- `naprava_id`
- `kot_stopinje`
- `casovna_zakasnitev_us`
- `raven_zvoka_db`
- `korelacijski_koeficient`

The field names are case-sensitive.

## Calibration is rejected

The response must contain valid numeric values for distance, correlation threshold, sound threshold, sample rate, and window size. The current firmware rejects remote sample rates other than 44100 and window sizes other than 1024.

Ambient temperature is intentionally not required in the response because the local BME280 supplies it.

## Measurements are dropped

The client retries an upload three times in RAM. If all attempts fail, the serial log reports that the measurement was dropped. Failed events are not persisted across reboot.

The two event queues have finite capacity. A sustained event rate can fill a queue and lose events because queue sends are non-blocking.
