#pragma once
#include <cstdint>
#include <cstddef>
#include <cstring>
#include <ctime>
#include <cmath>
#include <cstdlib>
#include <esp_rom_crc.h>
#include "audio/gcc_phat.h"
#include "config/app_config.h"

// Minimum plausible UTC timestamp (2024-01-01 00:00:00 UTC)
constexpr time_t   MIN_VALID_EPOCH                      = 1704067200;

// Binary REST Protocol Constants (POST /api/v1/measurements)
// Format version 1: 12-byte Little-Endian batch header + 20-byte per-measurement records + 4-byte CRC32 trailer
constexpr uint8_t  MEASUREMENT_BATCH_VERSION            = 1;
constexpr size_t   MEASUREMENT_BATCH_HEADER_SIZE        = 12;
constexpr size_t   MEASUREMENT_RECORD_SIZE              = 20;
constexpr size_t   MEASUREMENT_BATCH_CRC_SIZE           = 4;
constexpr size_t   MEASUREMENT_BATCH_TRAILER_SIZE       = MEASUREMENT_BATCH_CRC_SIZE;

// Clock synchronization flags
constexpr uint16_t CLOCK_FLAG_UTC_SYNCHRONIZED          = 0x0001; // Synchronized via SNTP UTC
constexpr uint16_t CLOCK_FLAG_SERVER_SYNCHRONIZED       = 0x0002; // Synchronized via API server response
constexpr uint16_t CLOCK_FLAG_MONOTONIC_UPTIME          = 0x0004; // Unsynchronized monotonic boot uptime

struct NetworkMeasurement {
    DoaResult result;
    uint64_t timestamp_ms;
    uint16_t clock_flags;
};

/**
 * @brief Disciplines the active SNTP UTC offset against a new gettimeofday() reading.
 *
 * @param current_offset_ms Active atomic ntp_utc_offset_ms.
 * @param measured_offset_ms Newly sampled (utc_ms - monotonic_ms).
 * @param max_plausible_drift_ms Maximum plausible drift limit (default: MAX_PLAUSIBLE_SNTP_DRIFT_MS).
 * @return Disciplined offset to store in ntp_utc_offset_ms.
 */
inline int64_t discipline_sntp_offset(int64_t current_offset_ms,
                                      int64_t measured_offset_ms,
                                      int64_t max_plausible_drift_ms = MAX_PLAUSIBLE_SNTP_DRIFT_MS) {
    const int64_t drift_ms = measured_offset_ms - current_offset_ms;
    if (std::abs(drift_ms) <= max_plausible_drift_ms) {
        return measured_offset_ms;
    }
    return current_offset_ms;
}

/**
 * @brief Disciplines the Server UTC offset using RTT gating and an EMA low-pass filter
 *        to reject Wi-Fi latency jitter while eliminating long-term crystal drift.
 *
 * @param current_offset_ms Active atomic server_utc_offset_ms.
 * @param measured_offset_ms Measured (server_time_ms - midpoint_ms).
 * @param rtt_ms Request round-trip time in milliseconds (request_end - request_start).
 * @param max_rtt_ms Maximum RTT threshold for clean time anchoring (default: MAX_CALIBRATION_RTT_MS).
 * @return Disciplined offset to store in server_utc_offset_ms.
 */
inline int64_t discipline_server_offset(int64_t current_offset_ms,
                                        int64_t measured_offset_ms,
                                        int64_t rtt_ms,
                                        int64_t max_rtt_ms = MAX_CALIBRATION_RTT_MS) {
    if (rtt_ms < 0 || rtt_ms > max_rtt_ms) {
        return current_offset_ms;
    }

    const int64_t diff_ms = measured_offset_ms - current_offset_ms;

    // If the server wall-clock experienced a large step (> 10s, e.g. server NTP sync),
    // commit directly without waiting for EMA convergence.
    if (std::abs(diff_ms) > 10000LL) {
        return measured_offset_ms;
    }

    if (diff_ms == 0) {
        return current_offset_ms;
    }

    // Smooth EMA low-pass filter (alpha = 0.25) to reject Wi-Fi RTT jitter
    int64_t step = diff_ms / 4LL;
    if (step == 0) {
        step = (diff_ms > 0) ? 1LL : -1LL;
    }

    return current_offset_ms + step;
}

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
 * @brief Calculates the IEEE 802.3 CRC32 checksum of a byte buffer using hardware ROM functions.
 *
 * @param data Pointer to input data.
 * @param length Number of bytes.
 * @return 32-bit CRC32 checksum (0 if data is null or length is 0).
 */
inline uint32_t calculate_batch_crc32(const uint8_t* data, size_t length) {
    if (data == nullptr || length == 0) {
        return 0;
    }
    return esp_rom_crc32_le(0, data, static_cast<uint32_t>(length));
}

/**
 * @brief Verifies the 4-byte CRC32 trailer appended to a serialized measurement batch.
 *
 * @param payload Pointer to serialized binary batch.
 * @param total_length Total size of payload in bytes (must be >= MEASUREMENT_BATCH_HEADER_SIZE + MEASUREMENT_BATCH_CRC_SIZE).
 * @return true if CRC matches; false otherwise.
 */
inline bool verify_measurement_batch_crc32(const uint8_t* payload, size_t total_length) {
    if (payload == nullptr || total_length < MEASUREMENT_BATCH_HEADER_SIZE + MEASUREMENT_BATCH_CRC_SIZE) {
        return false;
    }
    const size_t data_length = total_length - MEASUREMENT_BATCH_CRC_SIZE;
    const uint32_t expected_crc = calculate_batch_crc32(payload, data_length);

    uint32_t trailer_crc = 0;
    trailer_crc |= static_cast<uint32_t>(payload[data_length]);
    trailer_crc |= static_cast<uint32_t>(payload[data_length + 1]) << 8;
    trailer_crc |= static_cast<uint32_t>(payload[data_length + 2]) << 16;
    trailer_crc |= static_cast<uint32_t>(payload[data_length + 3]) << 24;

    return expected_crc == trailer_crc;
}

/**
 * @brief Serializes a batch of up to NETWORK_BATCH_MAX_MEASUREMENTS into the binary protocol format.
 *
 * @param measurements Array of timestamped measurements.
 * @param count Number of measurements in the batch (1..NETWORK_BATCH_MAX_MEASUREMENTS).
 * @param payload Destination buffer.
 * @param payload_capacity Total capacity of payload buffer.
 * @param db_spl_offset Calibration offset added to digital rms_db to form sound pressure level.
 * @return Total serialized bytes written (12 + count * 20 + 4), or 0 on validation error.
 */
inline size_t serialize_measurement_batch(
    const NetworkMeasurement* measurements,
    size_t count,
    uint8_t* payload,
    size_t payload_capacity,
    float db_spl_offset = 0.0f) {

    if (measurements == nullptr || payload == nullptr || count == 0 ||
        count > NETWORK_BATCH_MAX_MEASUREMENTS ||
        payload_capacity < MEASUREMENT_BATCH_HEADER_SIZE + count * MEASUREMENT_RECORD_SIZE + MEASUREMENT_BATCH_CRC_SIZE) {
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

    const uint32_t crc = calculate_batch_crc32(payload, offset);
    append_u32_le(payload, offset, crc);

    return offset;
}
