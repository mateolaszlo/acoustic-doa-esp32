#include "post_checker.h"
#include "app_config.h"
#include <cmath>

MicrophoneHealthResult evaluate_microphone_health(const int32_t* left,
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

    float l_mean = 0.0f;
    float r_mean = 0.0f;
    for (size_t i = 0; i < count; ++i) {
        l_mean += static_cast<float>(left[i]);
        r_mean += static_cast<float>(right[i]);
    }
    l_mean /= static_cast<float>(count);
    r_mean /= static_cast<float>(count);

    float l_ac_sum = 0.0f;
    float r_ac_sum = 0.0f;

    for (size_t i = 0; i < count; ++i) {
        if (left[i] < result.left_min) result.left_min = left[i];
        if (left[i] > result.left_max) result.left_max = left[i];
        if (right[i] < result.right_min) result.right_min = right[i];
        if (right[i] > result.right_max) result.right_max = right[i];

        if (left[i] == right[i]) {
            result.identical_samples++;
        }

        const float l_ac = (static_cast<float>(left[i]) - l_mean) / PCM_INT32_TO_FLOAT_SCALE;
        const float r_ac = (static_cast<float>(right[i]) - r_mean) / PCM_INT32_TO_FLOAT_SCALE;
        l_ac_sum += l_ac * l_ac;
        r_ac_sum += r_ac * r_ac;
    }

    const float l_rms = std::sqrt(l_ac_sum / static_cast<float>(count));
    const float r_rms = std::sqrt(r_ac_sum / static_cast<float>(count));

    result.left_rms_db = (l_rms > 1e-5f) ? 20.0f * std::log10(l_rms) : -100.0f;
    result.right_rms_db = (r_rms > 1e-5f) ? 20.0f * std::log10(r_rms) : -100.0f;

    // A valid microphone must:
    // 1. Have active variance (min != max)
    // 2. Not be completely silent/floating (RMS > POST_MIC_FLATLINE_RMS_DB)
    // 3. Not be pegged against the rail (RMS < POST_MIC_RAIL_SHORT_RMS_DB)
    result.left_ok = (result.left_min != result.left_max) &&
                     (result.left_rms_db > POST_MIC_FLATLINE_RMS_DB) &&
                     (result.left_rms_db < POST_MIC_RAIL_SHORT_RMS_DB);

    result.right_ok = (result.right_min != result.right_max) &&
                      (result.right_rms_db > POST_MIC_FLATLINE_RMS_DB) &&
                      (result.right_rms_db < POST_MIC_RAIL_SHORT_RMS_DB);

    // In true independent stereo audio, identical samples over a large block
    // should not exceed POST_MIC_MAX_COLLISION_PCT (e.g. 85%)
    result.stereo_distinct = (result.identical_samples * 100 < count * POST_MIC_MAX_COLLISION_PCT);

    return result;
}
