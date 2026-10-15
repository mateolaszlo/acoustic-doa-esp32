# Hardware and wiring

## Main board

The firmware targets an **ESP32-S3-DevKitC-1-N16R8**: 16 MB flash, 8 MB OPI PSRAM, two USB ports, and an on-board WS2812 RGB LED.

The board provides the CPU, Wi-Fi, I2S peripheral, I2C bus, GPIO 0 setup-button input, and two USB connections. The current firmware does not use the WS2812 LED.

The project includes a custom PlatformIO board definition at [boards/esp32-s3-devkitc-1-n16r8.json](../boards/esp32-s3-devkitc-1-n16r8.json), so the N16R8 flash, partition, Arduino variant, and OPI PSRAM settings are explicit rather than inherited from the generic N8 definition.

## Pinout

| Function | ESP32-S3 GPIO | Firmware definition |
|---|---:|---|
| I2S bit clock (BCLK/SCK) | GPIO 4 | `I2S_BCLK_PIN` |
| I2S word select (WS/LRCLK) | GPIO 5 | `I2S_WS_PIN` |
| I2S microphone data | GPIO 6 | `I2S_DIN_PIN` |
| I2C SDA (BME280) | GPIO 8 | `I2C_SDA_PIN` |
| I2C SCL (BME280) | GPIO 9 | `I2C_SCL_PIN` |
| SPI TFT CS | GPIO 10 | `TFT_CS_PIN` |
| SPI TFT MOSI / SDA | GPIO 11 | `TFT_MOSI_PIN` |
| SPI TFT SCLK / SCK | GPIO 12 | `TFT_SCLK_PIN` |
| SPI TFT DC / A0 | GPIO 13 | `TFT_DC_PIN` |
| SPI TFT RST / RESET | GPIO 14 | `TFT_RST_PIN` |
| Setup button | GPIO 0 | `SETUP_BUTTON_PIN` |

> [!CAUTION]
> **OPI PSRAM Pin Restrictions:** On the ESP32-S3-DevKitC-1-N16R8, GPIO 33, 34, 35, 36, and 37 are internally dedicated to the 8-bit Octal SPI PSRAM. Never connect display or sensor signals to these pins, as doing so causes instant memory corruption or kernel panics.

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

## SPI TFT Display Wiring (1.8" 128x160 KMR-1.8 / ST7735)

The display uses native ESP32-S3 FSPI (SPI2) pins for hardware-accelerated rendering up to 40 MHz:

| KMR-1.8 Pin | ESP32-S3 | Function | Notes |
|---|---|---|---|
| **CS** | GPIO 10 | Chip Select | Native FSPI CS0 |
| **SDA / MOSI** | GPIO 11 | Master Out Slave In | Native FSPI D |
| **SCK / SCLK** | GPIO 12 | Clock | Native FSPI CLK |
| **A0 / DC** | GPIO 13 | Data / Command | Native FSPI Q pin repurposed for DC |
| **RESET / RST** | GPIO 14 | Reset | Active-low hardware reset |
| **LED** | 3.3 V | Backlight | Connect to 3.3V (or via current-limiting resistor) |
| **GND** | GND | Ground | Common system ground |
| **VCC** | 5 V (or 3.3 V) | Power input | Connect to 5V (VIN) if jumper J1 is open, or 3.3V if J1 is bridged |

> [!TIP]
> **KMR-1.8 Jumper J1:** The KMR-1.8 board includes an onboard 3.3V LDO regulator. If feeding 3.3V to VCC without closing jumper `J1`, the voltage drop across the regulator may cause a faint screen or initialization failure. Connect VCC to the 5V rail (VIN) of the DevKit, or bridge `J1` with solder to power directly from 3.3V.

## BME280 I2C Wiring

The Bosch BME280 environmental sensor communicates over I2C:

| I2C signal | ESP32-S3 | BME280 |
|---|---:|---|
| SDA | GPIO 8 | SDA |
| SCL | GPIO 9 | SCL |
| Ground | GND | GND |
| Power | 3.3 V | VIN/VDD |

The bus is initialized at 400 kHz Fast I2C. The BME280 is probed first at `0x76`, then at `0x77`.

## Setup button

GPIO 0 is configured with `INPUT_PULLUP`; the setup button is active-low and should connect GPIO 0 to GND when pressed.

GPIO 0 is also a boot-strapping pin on ESP32-S3 boards. Holding it low while pressing reset or powering on can select the ROM download bootloader instead of the application. Follow the timing in [Build, provisioning, and setup](setup.md), or use a different non-strapping GPIO if the hardware design can be changed.

## Power and layout

- Use a stable 3.3 V supply suitable for the ESP32 and both microphones.
- Keep microphone ground and ESP32 ground common.
- Keep the two microphone capsules mechanically fixed relative to one another.
- Measure the microphone-center spacing accurately; the distance is used directly in the DOA equation.
- Keep I2S wires short and routed away from noisy switching or display wiring where practical.
