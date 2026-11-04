#pragma once
#include <string>
#include <map>
#include <cstdint>
#include <cmath>
#include <cstdlib>
#include <cstdio>

#ifndef ARDUINO
using String = std::string;
#endif

class Preferences {
public:
    static size_t total_writes;
    static std::map<std::string, std::string> mock_storage;

    bool begin(const char* name, bool read_only = false) {
        (void)name;
        (void)read_only;
        open_ = true;
        return true;
    }

    void end() {
        open_ = false;
    }

    bool isKey(const char* key) const {
        if (!key) return false;
        return mock_storage.find(key) != mock_storage.end();
    }

    size_t putString(const char* key, const String& val) {
        if (!key) return 0;
        mock_storage[key] = val;
        total_writes++;
        return val.length();
    }

    String getString(const char* key, const String& default_val = String()) const {
        if (!key || mock_storage.find(key) == mock_storage.end()) {
            return default_val;
        }
        return mock_storage.at(key);
    }

    size_t putFloat(const char* key, float val) {
        if (!key) return 0;
        char buf[64];
        snprintf(buf, sizeof(buf), "%.8g", static_cast<double>(val));
        mock_storage[key] = buf;
        total_writes++;
        return sizeof(float);
    }

    float getFloat(const char* key, float default_val = NAN) const {
        if (!key || mock_storage.find(key) == mock_storage.end()) {
            return default_val;
        }
        return static_cast<float>(strtod(mock_storage.at(key).c_str(), nullptr));
    }

    size_t putUInt(const char* key, uint32_t val) {
        if (!key) return 0;
        mock_storage[key] = std::to_string(val);
        total_writes++;
        return sizeof(uint32_t);
    }

    uint32_t getUInt(const char* key, uint32_t default_val = 0) const {
        if (!key || mock_storage.find(key) == mock_storage.end()) {
            return default_val;
        }
        return static_cast<uint32_t>(strtoul(mock_storage.at(key).c_str(), nullptr, 10));
    }

    size_t putBool(const char* key, bool val) {
        if (!key) return 0;
        mock_storage[key] = val ? "1" : "0";
        total_writes++;
        return sizeof(bool);
    }

    bool getBool(const char* key, bool default_val = false) const {
        if (!key || mock_storage.find(key) == mock_storage.end()) {
            return default_val;
        }
        return mock_storage.at(key) == "1";
    }

    static void resetMockStats() {
        total_writes = 0;
    }

    static void clearStorage() {
        mock_storage.clear();
        total_writes = 0;
    }

private:
    bool open_ = false;
};
