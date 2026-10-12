# Hardware and wiring

## Main board

The firmware targets an **ESP32-S3-DevKitC-1-N8** using the `esp32-s3-devkitc-1` PlatformIO board definition.

The board provides the CPU, Wi-Fi, I2S peripheral, I2C bus, GPIO 0 setup-button input, and USB/serial connection.

## Pinout

| Function | ESP32-S3 GPIO | Firmware definition |
|---|---:|---|
| I2S bit clock (BCLK/SCK) | GPIO 4 | `I2S_BCLK_PIN` |
| I2S word select (WS/LRCLK) | GPIO 5 | `I2S_WS_PIN` |
| I2S microphone data | GPIO 6 | `I2S_DIN_PIN` |
| I2C SDA | GPIO 8 | `I2C_SDA_PIN` |
| I2C SCL | GPIO 9 | `I2C_SCL_PIN` |
| Setup button | GPIO 0 | `SETUP_BUTTON_PIN` |

GPIO numbers and peripheral setup are defined in [main.cpp](../src/main.cpp) and [i2s_sampler.cpp](../src/i2s_sampler.cpp).

## Two-microphone I2S wiring

The firmware expects two synchronized digital microphones connected to one shared I2S bus:

- Both microphones share BCLK.
- Both microphones share WS/LRCLK.
- Both microphones share the data input line into GPIO 6.
- One microphone must transmit in the left slot.
- The other microphone must transmit in the right slot.

| I2S signal | ESP32-S3 | Microphone 1 | Microphone 2 |
|---|---:|---|---|
| BCLK | GPIO 4 | BCLK/SCK | BCLK/SCK |
| WS/LRCLK | GPIO 5 | WS/LRCLK | WS/LRCLK |
| DATA | GPIO 6 | SD/data bus | SD/data bus |
| Channel select | N/A | Left | Right |
| Ground | GND | GND | GND |
| Power | 3.3 V as required by the module | VDD | VDD |

The I2S driver is configured as master receive, 32-bit samples, stereo left/right format, four DMA buffers, and DMA buffer length 512.

Confirm the microphone module voltage and slot-selection wiring from its datasheet before connecting it. Do not connect a microphone that drives a separate data line directly to the same ESP32 data input without an appropriate bus design.

## OLED and BME280 I2C wiring

The OLED and BME280 share the same I2C bus:

| I2C signal | ESP32-S3 | OLED | BME280 |
|---|---:|---|---|
| SDA | GPIO 8 | SDA | SDA |
| SCL | GPIO 9 | SCL | SCL |
| Ground | GND | GND | GND |
| Power | 3.3 V as required | VCC | VIN/VDD |

The bus is initialized at 400 kHz.

The OLED is expected at address `0x3C`.

The BME280 is probed first at `0x76`, then at `0x77`.

Both I2C devices must have compatible voltage levels. Pull-up resistors may already be present on breakout boards; avoid adding excessive parallel pull-ups.

## Setup button

GPIO 0 is configured with `INPUT_PULLUP`; the setup button is active-low and should connect GPIO 0 to GND when pressed.

GPIO 0 is also a boot-strapping pin on ESP32-S3 boards. Holding it low while pressing reset or powering on can select the ROM download bootloader instead of the application. Follow the timing in [Build, provisioning, and setup](setup.md), or use a different non-strapping GPIO if the hardware design can be changed.

## Power and layout

- Use a stable 3.3 V supply suitable for the ESP32 and both microphones.
- Keep microphone ground and ESP32 ground common.
- Keep the two microphone capsules mechanically fixed relative to one another.
- Measure the microphone-center spacing accurately; the distance is used directly in the DOA equation.
- Keep I2S wires short and routed away from noisy switching or display wiring where practical.
