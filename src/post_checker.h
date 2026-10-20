#ifndef POST_CHECKER_H
#define POST_CHECKER_H

#include <cstdint>
#include <cstddef>

/**
 * @brief Results of statistical heuristic evaluation of stereo digital microphones.
 */
struct MicrophoneHealthResult {
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
};

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

#endif // POST_CHECKER_H
