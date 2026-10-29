#pragma once
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Software emulation of the ESP32-S3 ROM CRC32 function for host testing.
 *
 * Implements the standard IEEE 802.3 CRC32 (polynomial 0x04C11DB7, bit-reflected: 0xEDB88320).
 * Initial call passes crc = 0, which is inverted internally (~0 = 0xFFFFFFFF) matching
 * standard esp_rom_crc32_le behavior.
 */
static inline uint32_t esp_rom_crc32_le(uint32_t crc, const uint8_t *buf, uint32_t len) {
    crc = ~crc;
    while (len--) {
        crc ^= *buf++;
        for (int k = 0; k < 8; ++k) {
            crc = (crc >> 1) ^ (0xEDB88320U & (-(crc & 1U)));
        }
    }
    return ~crc;
}

static inline uint32_t esp_crc32_le(uint32_t crc, const uint8_t *buf, uint32_t len) {
    return esp_rom_crc32_le(crc, buf, len);
}

#ifdef __cplusplus
}
#endif
