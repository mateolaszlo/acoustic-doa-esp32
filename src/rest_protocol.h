#pragma once
#include <cstdint>
#include <cstddef>
#include <cstring>
#include <ctime>
#include "gcc_phat.h"
#include "app_config.h"

// Minimum plausible UTC timestamp (2024-01-01 00:00:00 UTC)
constexpr time_t   MIN_VALID_EPOCH                      = 1704067200;

// Binary REST Protocol Constants (POST /api/v1/meritve)
// Format version 1: 12-byte Little-Endian batch header + 20-byte per-measurement records
constexpr uint8_t  MEASUREMENT_BATCH_VERSION            = 1;
constexpr size_t   MEASUREMENT_BATCH_HEADER_SIZE        = 12;
constexpr size_t   MEASUREMENT_RECORD_SIZE              = 20;

// Clock synchronization flags
constexpr uint16_t CLOCK_FLAG_UTC_SYNCHRONIZED          = 0x0001; // Synchronized via SNTP UTC
constexpr uint16_t CLOCK_FLAG_SERVER_SYNCHRONIZED       = 0x0002; // Synchronized via API server response
constexpr uint16_t CLOCK_FLAG_MONOTONIC_UPTIME          = 0x0004; // Unsynchronized monotonic boot uptime

struct NetworkMeasurement {
    DoaResult result;
    uint64_t timestamp_ms;
    uint16_t clock_flags;
};

inline void append_u16_le(uint8_t* buffer, size_t &offset, uint16_t value) {
    buffer[offset++] = static_cast<uint8_t>(value);
    buffer[offset++] = static_cast<uint8_t>(value >> 8);
}

inline void append_u32_le(uint8_t* buffer, size_t &offset, uint32_t value) {
    for (size_t byte = 0; byte < sizeof(value); ++byte) {
        buffer[offset++] = static_cast<uint8_t>(value >> (byte * 8));
    }
}

inline void append_u64_le(uint8_t* buffer, size_t &offset, uint64_t value) {
    for (size_t byte = 0; byte < sizeof(value); ++byte) {
        buffer[offset++] = static_cast<uint8_t>(value >> (byte * 8));
    }
}

inline void append_float_le(uint8_t* buffer, size_t &offset, float value) {
    static_assert(sizeof(float) == sizeof(uint32_t), "Binary protocol requires 32-bit floats");
    uint32_t bits = 0;
    std::memcpy(&bits, &value, sizeof(bits));
    append_u32_le(buffer, offset, bits);
}

/**
 * @brief Serializes a batch of up to NETWORK_BATCH_MAX_MEASUREMENTS into the binary protocol format.
 *
 * @param measurements Array of timestamped measurements.
 * @param count Number of measurements in the batch (1..NETWORK_BATCH_MAX_MEASUREMENTS).
 * @param payload Destination buffer.
 * @param payload_capacity Total capacity of payload buffer.
 * @param db_spl_offset Calibration offset added to digital rms_db to form sound pressure level.
 * @return Total serialized bytes written (12 + count * 20), or 0 on validation error.
 */
inline size_t serialize_measurement_batch(
    const NetworkMeasurement* measurements,
    size_t count,
    uint8_t* payload,
    size_t payload_capacity,
    float db_spl_offset = 0.0f) {

    if (measurements == nullptr || payload == nullptr || count == 0 ||
        count > NETWORK_BATCH_MAX_MEASUREMENTS ||
        payload_capacity < MEASUREMENT_BATCH_HEADER_SIZE + count * MEASUREMENT_RECORD_SIZE) {
        return 0;
    }

    const uint64_t base_timestamp_ms = measurements[0].timestamp_ms;
    const uint16_t clock_flags = measurements[0].clock_flags;
    if (base_timestamp_ms == 0 || clock_flags == 0) {
        return 0;
    }

    size_t offset = 0;
    payload[offset++] = MEASUREMENT_BATCH_VERSION;
    payload[offset++] = static_cast<uint8_t>(count);
    append_u16_le(payload, offset, clock_flags);
    append_u64_le(payload, offset, base_timestamp_ms);

    for (size_t index = 0; index < count; ++index) {
        // Enforce consistent clock source and strictly non-decreasing timestamps within a single batch
        if (measurements[index].clock_flags != clock_flags ||
            measurements[index].timestamp_ms < base_timestamp_ms ||
            measurements[index].timestamp_ms - base_timestamp_ms > UINT32_MAX) {
            return 0;
        }

        const uint32_t delta_ms = static_cast<uint32_t>(measurements[index].timestamp_ms - base_timestamp_ms);
        append_u32_le(payload, offset, delta_ms);
        append_float_le(payload, offset, measurements[index].result.angle_deg);
        append_float_le(payload, offset, measurements[index].result.delay_us);
        append_float_le(payload, offset, measurements[index].result.rms_db + db_spl_offset);
        append_float_le(payload, offset, measurements[index].result.confidence);
    }

    return offset;
}
