#pragma once
#include <cstdint>
#include "Adafruit_GFX.h"
#include "SPI.h"

#define INITR_BLACKTAB 0x00

class Adafruit_ST7735 : public Adafruit_GFX {
public:
    Adafruit_ST7735(SPIClass* spi, int8_t cs, int8_t dc, int8_t rst)
        : Adafruit_GFX(128, 160), spi_port(spi), cs_pin(cs), dc_pin(dc), rst_pin(rst) {}
    ~Adafruit_ST7735() override {}

    void drawPixel(int16_t x, int16_t y, uint16_t color) override {
        (void)x; (void)y; (void)color;
    }
    void initR(uint8_t options = INITR_BLACKTAB) { (void)options; }
    void setSPISpeed(uint32_t freq) { (void)freq; }
    void setRotation(uint8_t r) { (void)r; }
    void invertDisplay(bool i) { (void)i; }
    void drawRGBBitmap(int16_t x, int16_t y, const uint16_t* bitmap, int16_t w, int16_t h) {
        (void)x; (void)y; (void)bitmap; (void)w; (void)h;
    }

private:
    SPIClass* spi_port;
    int8_t cs_pin, dc_pin, rst_pin;
};
