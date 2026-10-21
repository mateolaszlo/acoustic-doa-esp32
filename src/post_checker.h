#ifndef POST_CHECKER_H
#define POST_CHECKER_H

#include <cstdint>
#include <cstddef>
#include "app_config.h"

/**
 * @brief Results of statistical heuristic evaluation of stereo or quad digital microphones.
 */
struct MicrophoneHealthResult {
    // Legacy 2-channel fields (mapped to Bus 0: Mic 0 Left, Mic 1 Right)
    bool left_ok;
    bool right_ok;
    bool stereo_distinct;
    float left_rms_db;
    float right_rms_db;
    int32_t left_min;
    int32_t left_max;
    int32_t right_min;
    int32_t right_max;
    size_t identical_samples;

    // 4-channel planar square array fields (Mic 0..3)
    bool ch_ok[AUDIO_NUM_CHANNELS];
    float ch_rms_db[AUDIO_NUM_CHANNELS];
    int32_t ch_min[AUDIO_NUM_CHANNELS];
    int32_t ch_max[AUDIO_NUM_CHANNELS];
    bool bus0_distinct;      // Intra-bus 0: Mic 0 (L) vs Mic 1 (R)
    bool bus1_distinct;      // Intra-bus 1: Mic 2 (L) vs Mic 3 (R)
    bool cross_bus_distinct; // Inter-bus: Bus 0 (GPIO 6) vs Bus 1 (GPIO 7)
    bool all_ok;             // All 4 channels healthy and all pairs distinct
};

using QuadMicrophoneHealthResult = MicrophoneHealthResult;

/**
 * @brief Evaluates acoustic and electrical health of stereo digital microphones
 *        from a raw I2S sample block.
 *
 * Diagnoses:
 * - Disconnected or unpowered microphones (flatline: min == max, RMS <= -95 dBFS).
 * - DC shorts to power rail (RMS >= -3 dBFS).
 * - Channel collision (identical Left and Right samples across the block).
 */
MicrophoneHealthResult evaluate_microphone_health(const int32_t* left,
                                                 const int32_t* right,
                                                 size_t count);

/**
 * @brief Evaluates acoustic and electrical health of all 4 digital microphones
 *        across dual I2S buses (Bus 0: ch0/ch1, Bus 1: ch2/ch3).
 *
 * Diagnoses:
 * - Per-channel variance and flatline (min == max or RMS <= -95 dBFS).
 * - Per-channel DC or AC shorts to power rail (RMS >= -3 dBFS).
 * - Intra-bus L/R collision on Bus 0 (ch0 vs ch1) and Bus 1 (ch2 vs ch3).
 * - Cross-bus data line shorting between Bus 0 and Bus 1.
 */
MicrophoneHealthResult evaluate_quad_microphone_health(
    const int32_t* const channels[AUDIO_NUM_CHANNELS],
    size_t count);

MicrophoneHealthResult evaluate_quad_microphone_health(
    const int32_t* ch0,
    const int32_t* ch1,
    const int32_t* ch2,
    const int32_t* ch3,
    size_t count);

#endif // POST_CHECKER_H
