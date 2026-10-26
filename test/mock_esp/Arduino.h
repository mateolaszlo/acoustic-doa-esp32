#pragma once
#include <cstdint>
#include <cstddef>
#include <cstdio>
#include <cstdarg>
#include <string>

#ifndef ARDUINO
using String = std::string;
uint32_t millis();

class HardwareSerial {
public:
    void print(const char* s) {
        if (s) buffer += s;
    }
    void print(const String& s) {
        buffer += s;
    }
    void println(const char* s = "") {
        if (s) buffer += s;
        buffer += "\n";
    }
    void println(const String& s) {
        buffer += s + "\n";
    }
    void printf(const char* format, ...) {
        char temp[2048];
        va_list args;
        va_start(args, format);
        vsnprintf(temp, sizeof(temp), format, args);
        va_end(args);
        buffer += temp;
    }
    const std::string& getOutput() const { return buffer; }
    void clear() { buffer.clear(); }

private:
    std::string buffer;
};

extern HardwareSerial Serial;
#endif
