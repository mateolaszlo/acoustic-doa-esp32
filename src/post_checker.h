#ifndef POST_CHECKER_H
#define POST_CHECKER_H

#include <cmath>
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
inline MicrophoneHealthResult evaluate_microphone_health(const int32_t* left,
                                                        const int32_t* right,
                                                        size_t count) {
    MicrophoneHealthResult result;
    result.left_ok = false;
    result.right_ok = false;
    result.stereo_distinct = false;
    result.left_rms_db = -100.0f;
    result.right_rms_db = -100.0f;
    result.left_min = 0;
    result.left_max = 0;
    result.right_min = 0;
    result.right_max = 0;
    result.identical_samples = 0;

    if (left == nullptr || right == nullptr || count == 0) {
        return result;
    }

    result.left_min = left[0];
    result.left_max = left[0];
    result.right_min = right[0];
    result.right_max = right[0];

    float l_sum = 0.0f;
    float r_sum = 0.0f;

    for (size_t i = 0; i < count; ++i) {
        if (left[i] < result.left_min) result.left_min = left[i];
        if (left[i] > result.left_max) result.left_max = left[i];
        if (right[i] < result.right_min) result.right_min = right[i];
        if (right[i] > result.right_max) result.right_max = right[i];

        if (left[i] == right[i]) {
            result.identical_samples++;
        }

        const float l_norm = static_cast<float>(left[i]) / 2147483648.0f;
        const float r_norm = static_cast<float>(right[i]) / 2147483648.0f;
        l_sum += l_norm * l_norm;
        r_sum += r_norm * r_norm;
    }

    const float l_rms = std::sqrt(l_sum / static_cast<float>(count));
    const float r_rms = std::sqrt(r_sum / static_cast<float>(count));

    result.left_rms_db = (l_rms > 1e-5f) ? 20.0f * std::log10(l_rms) : -100.0f;
    result.right_rms_db = (r_rms > 1e-5f) ? 20.0f * std::log10(r_rms) : -100.0f;

    // A valid microphone must:
    // 1. Have active variance (min != max)
    // 2. Not be completely silent/floating (RMS > -95 dBFS)
    // 3. Not be pegged against the rail (RMS < -3 dBFS)
    result.left_ok = (result.left_min != result.left_max) &&
                     (result.left_rms_db > -95.0f) &&
                     (result.left_rms_db < -3.0f);

    result.right_ok = (result.right_min != result.right_max) &&
                      (result.right_rms_db > -95.0f) &&
                      (result.right_rms_db < -3.0f);

    // In true independent stereo audio, identical samples over a large block
    // should not exceed 85% of samples (e.g. 870 out of 1024)
    result.stereo_distinct = (result.identical_samples * 100 < count * 85);

    return result;
}

#endif // POST_CHECKER_H
