#pragma once
#include <cstdint>

#define FSPI 2

class SPIClass {
public:
    explicit SPIClass(int bus = FSPI) : bus_num(bus) {}
    void begin(int sclk, int miso, int mosi, int ss) {
        (void)sclk; (void)miso; (void)mosi; (void)ss;
    }
private:
    int bus_num;
};
