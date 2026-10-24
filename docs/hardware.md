# Hardware and wiring

## Main board

The firmware targets an **ESP32-S3-DevKitC-1-N16R8**: 16 MB flash, 8 MB OPI PSRAM, two USB ports, and an on-board WS2812 RGB LED.

The board provides the CPU, Wi-Fi, I2S peripheral, I2C bus, GPIO 0 setup-button input, and two USB connections. The current firmware does not use the WS2812 LED.

The project includes a custom PlatformIO board definition at [boards/esp32-s3-devkitc-1-n16r8.json](../boards/esp32-s3-devkitc-1-n16r8.json), so the N16R8 flash, partition, Arduino variant, and OPI PSRAM settings are explicit rather than inherited from the generic N8 definition.

## Pinout

| Function | ESP32-S3 GPIO | Firmware definition |
|---|---:|---|
| I2S bit clock (BCLK/SCK, shared Bus 0 & Bus 1) | GPIO 5 | `I2S_BCLK_PIN` |
| I2S word select (WS/LRCLK, shared Bus 0 & Bus 1) | GPIO 4 | `I2S_WS_PIN` |
| I2S Bus 0 microphone data (Mic 0 & Mic 1) | GPIO 6 | `I2S_DIN0_PIN` (`I2S_DIN_PIN`) |
| I2S Bus 1 microphone data (Mic 2 & Mic 3) | GPIO 7 | `I2S_DIN1_PIN` |
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

GPIO numbers and peripheral setup are defined in [app_config.h](../src/config/app_config.h), [main.cpp](../src/main.cpp), and [i2s_sampler.cpp](../src/audio/i2s_sampler.cpp).

## Four-microphone 2D square array I2S wiring

The firmware uses four synchronized digital I2S microphones (e.g., INMP441) arranged in a horizontal 2D planar square of side length $d = 82\text{ mm}$ (`DEFAULT_MIC_DISTANCE_M = 0.082f`) to resolve full $360^\circ$ azimuth ($\phi \in [-180^\circ, +180^\circ]$, where $0^\circ = \text{Front/North/+Y}$, $+90^\circ = \text{Right/East/+X}$, $\pm 180^\circ = \text{Back/South/-Y}$, and $-90^\circ = \text{Left/West/-X}$):

- **Shared Clock Bus**: All four microphones share `BCLK` (`GPIO 5`) and `WS/LRCLK` (`GPIO 4`).
- **Dual ESP32-S3 I2S Controllers via GPIO Matrix**:
  - `I2S0` (`I2S_MASTER_PORT_NUM = 0`) operates as Master RX, driving `BCLK` on `GPIO 5` and `WS` on `GPIO 4` while sampling stereo Pair A (`Mic 0` and `Mic 1`) on `GPIO 6` (`I2S_DIN0_PIN`).
  - `I2S1` (`I2S_SLAVE_PORT_NUM = 1`) operates as Slave RX, sampling stereo Pair B (`Mic 2` and `Mic 3`) on `GPIO 7` (`I2S_DIN1_PIN`).
  - `GPIO 5` and `GPIO 4` are configured in `GPIO_MODE_INPUT_OUTPUT` and routed internally through the ESP32-S3 GPIO Matrix (`I2S1I_BCK_IN_IDX` and `I2S1I_WS_IN_IDX`) so `I2S1` locks onto `I2S0`'s bit clock and frame sync without external loopback jumpers.
  - To guarantee zero inter-bus sample slip, `I2S1` (Slave) is armed via `i2s_start(I2S_NUM_1)` before `I2S0` (Master) starts outputting clocks via `i2s_start(I2S_NUM_0)`.

| Microphone | Array Corner | Coordinates $(x, y)$ | I2S Peripheral | Data Pin | L/R Slot (`L/R` Pin) |
|---|---|---|---|---:|---|
| **Mic 0 (`ch0`)** | Right-Back (RB) | $(+d/2, -d/2)$ | `I2S0` (Master RX) | GPIO 6 (`I2S_DIN0_PIN`) | Left (`GND`) |
| **Mic 1 (`ch1`)** | Right-Front (RF) | $(+d/2, +d/2)$ | `I2S0` (Master RX) | GPIO 6 (`I2S_DIN0_PIN`) | Right (`VDD`) |
| **Mic 2 (`ch2`)** | Left-Front (LF) | $(-d/2, +d/2)$ | `I2S1` (Slave RX) | GPIO 7 (`I2S_DIN1_PIN`) | Left (`GND`) |
| **Mic 3 (`ch3`)** | Left-Back (LB) | $(-d/2, -d/2)$ | `I2S1` (Slave RX) | GPIO 7 (`I2S_DIN1_PIN`) | Right (`VDD`) |

| I2S signal | ESP32-S3 | Mic 0 (RB) | Mic 1 (RF) | Mic 2 (LF) | Mic 3 (LB) |
|---|---:|---|---|---|---|
| BCLK | GPIO 5 | BCLK/SCK | BCLK/SCK | BCLK/SCK | BCLK/SCK |
| WS/LRCLK | GPIO 4 | WS/LRCLK | WS/LRCLK | WS/LRCLK | WS/LRCLK |
| DATA Bus 0 | GPIO 6 | SD/data | SD/data | — | — |
| DATA Bus 1 | GPIO 7 | — | — | SD/data | SD/data |
| Channel select (`L/R`) | N/A | Left (`GND`) | Right (`VDD`) | Left (`GND`) | Right (`VDD`) |
| Ground | GND | GND | GND | GND | GND |
| Power | 3.3 V | VDD | VDD | VDD | VDD |

Both I2S drivers are configured for 32-bit samples, stereo left/right standard I2S format, four DMA buffers, and DMA buffer length 512, followed by a synchronous 12-block warmup discard (`POST_WARMUP_BLOCKS = 12`, $\approx 278\text{ ms}$) to settle MEMS charge pumps.

Confirm the microphone module voltage and slot-selection wiring from its datasheet before connecting it. Do not short `GPIO 6` and `GPIO 7` together; the Power-On Self-Test (POST) explicitly checks for both intra-bus L/R collision and cross-bus data-line shorts.

## SPI TFT Display Wiring (1.8" 128x160 KMR-1.8 / ST7735)

The display uses native ESP32-S3 FSPI (SPI2) pins for hardware-accelerated rendering at 27 MHz:

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

- Use a stable 3.3 V supply suitable for the ESP32 and all four microphones.
- Keep microphone ground and ESP32 ground common.
- Keep the four microphone capsules mechanically fixed in a planar square of side length $d = 82\text{ mm}$ (`DEFAULT_MIC_DISTANCE_M = 0.082f`).
- Measure the edge spacing $d$ accurately; the baseline geometry is used directly in the 6-pair 2D WLS DOA solver.
- Keep I2S wires short and routed away from noisy switching or display wiring where practical.
