#pragma once
#include <cstdint>
#include <cstddef>
#include <cstdio>
#include <cstdarg>
#include <vector>

#define ST77XX_BLACK   0x0000
#define ST77XX_WHITE   0xFFFF
#define ST77XX_RED     0xF800
#define ST77XX_GREEN   0x07E0
#define ST77XX_CYAN    0x07FF
#define ST77XX_MAGENTA 0xF81F
#define ST77XX_YELLOW  0xFFE0
#define ST77XX_ORANGE  0xFD20

#ifndef ARDUINO
uint32_t millis();
#endif

class Adafruit_GFX {
public:
    Adafruit_GFX(int16_t w, int16_t h)
        : WIDTH(w), HEIGHT(h), cursor_x(0), cursor_y(0),
          textsize_x(1), textsize_y(1), textcolor(ST77XX_WHITE), textbgcolor(ST77XX_BLACK) {}
    virtual ~Adafruit_GFX() {}

    virtual void drawPixel(int16_t x, int16_t y, uint16_t color) = 0;
    virtual void fillScreen(uint16_t color) { (void)color; }
    virtual void drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color) {
        (void)x0; (void)y0; (void)x1; (void)y1; (void)color;
    }
    virtual void drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color) {
        (void)x; (void)y; (void)w; (void)color;
    }
    virtual void drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color) {
        (void)x; (void)y; (void)h; (void)color;
    }
    virtual void drawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) {
        (void)x; (void)y; (void)w; (void)h; (void)color;
    }
    virtual void fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) {
        (void)x; (void)y; (void)w; (void)h; (void)color;
    }
    virtual void drawCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color) {
        (void)x0; (void)y0; (void)r; (void)color;
    }
    virtual void fillCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color) {
        (void)x0; (void)y0; (void)r; (void)color;
    }
    virtual void drawRoundRect(int16_t x, int16_t y, int16_t w, int16_t h, int16_t r, uint16_t color) {
        (void)x; (void)y; (void)w; (void)h; (void)r; (void)color;
    }
    virtual void fillRoundRect(int16_t x, int16_t y, int16_t w, int16_t h, int16_t r, uint16_t color) {
        (void)x; (void)y; (void)w; (void)h; (void)r; (void)color;
    }

    void setCursor(int16_t x, int16_t y) { cursor_x = x; cursor_y = y; }
    void setTextColor(uint16_t c) { textcolor = c; textbgcolor = c; }
    void setTextColor(uint16_t c, uint16_t bg) { textcolor = c; textbgcolor = bg; }
    void setTextSize(uint8_t s) { textsize_x = s; textsize_y = s; }

    void print(const char* s) { (void)s; }
    void print(char c) { (void)c; }
    void print(int n) { (void)n; }
    void print(unsigned int n) { (void)n; }
    void print(float f) { (void)f; }
    void println(const char* s = "") { (void)s; }
    void printf(const char* format, ...) {
        va_list args;
        va_start(args, format);
        va_end(args);
    }

protected:
    int16_t WIDTH, HEIGHT;
    int16_t cursor_x, cursor_y;
    uint8_t textsize_x, textsize_y;
    uint16_t textcolor, textbgcolor;
};

class GFXcanvas16 : public Adafruit_GFX {
public:
    GFXcanvas16(uint16_t w, uint16_t h) : Adafruit_GFX(w, h), buffer(w * h, 0) {}
    ~GFXcanvas16() override {}

    void drawPixel(int16_t x, int16_t y, uint16_t color) override {
        if (x >= 0 && x < WIDTH && y >= 0 && y < HEIGHT) {
            buffer[y * WIDTH + x] = color;
        }
    }
    void fillScreen(uint16_t color) override {
        for (auto& p : buffer) p = color;
    }
    uint16_t* getBuffer() { return buffer.data(); }
    const uint16_t* getBuffer() const { return buffer.data(); }

private:
    std::vector<uint16_t> buffer;
};
