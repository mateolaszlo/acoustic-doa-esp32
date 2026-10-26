#pragma once
#include <cstdint>

#ifndef ARDUINO
class TwoWire {
public:
    void beginTransmission(uint8_t address) {
        last_address = address;
    }
    uint8_t endTransmission() {
        return (last_address == 0x76 || last_address == 0x77) ? 0 : 2;
    }
private:
    uint8_t last_address = 0;
};

extern TwoWire Wire;
#endif
