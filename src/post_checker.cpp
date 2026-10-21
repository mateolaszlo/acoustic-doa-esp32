#include "post_checker.h"
#include "app_config.h"
#include <cmath>

static void init_health_result(MicrophoneHealthResult& result) {
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

    for (size_t c = 0; c < AUDIO_NUM_CHANNELS; ++c) {
        result.ch_ok[c] = false;
        result.ch_rms_db[c] = -100.0f;
        result.ch_min[c] = 0;
        result.ch_max[c] = 0;
    }
    result.bus0_distinct = false;
    result.bus1_distinct = false;
    result.cross_bus_distinct = false;
    result.all_ok = false;
}

static void evaluate_single_channel(const int32_t* samples,
                                    size_t count,
                                    int32_t& out_min,
                                    int32_t& out_max,
                                    float& out_rms_db,
                                    bool& out_ok) {
    out_min = samples[0];
    out_max = samples[0];

    int64_t sum = 0;
    for (size_t i = 0; i < count; ++i) {
        sum += static_cast<int64_t>(samples[i]);
    }
    const double mean = static_cast<double>(sum) / static_cast<double>(count);
    const double scale = static_cast<double>(PCM_INT32_TO_FLOAT_SCALE);

    double ac_sum = 0.0;
    double raw_sum = 0.0;
    for (size_t i = 0; i < count; ++i) {
        if (samples[i] < out_min) out_min = samples[i];
        if (samples[i] > out_max) out_max = samples[i];

        const double sample_d = static_cast<double>(samples[i]);
        const double raw_norm = sample_d / scale;
        const double ac_norm = (sample_d - mean) / scale;
        raw_sum += raw_norm * raw_norm;
        ac_sum += ac_norm * ac_norm;
    }

    const float raw_rms = static_cast<float>(std::sqrt(raw_sum / static_cast<double>(count)));
    const float ac_rms = static_cast<float>(std::sqrt(ac_sum / static_cast<double>(count)));
    const float raw_rms_db = (raw_rms > 1e-5f) ? 20.0f * std::log10(raw_rms) : -100.0f;
    const float ac_rms_db = (ac_rms > 1e-5f) ? 20.0f * std::log10(ac_rms) : -100.0f;

    // Report raw RMS if the line is shorted to a power rail (>= -3 dBFS DC or AC);
    // otherwise report AC-coupled RMS for normal acoustic variance evaluation.
    out_rms_db = (raw_rms_db >= POST_MIC_RAIL_SHORT_RMS_DB) ? raw_rms_db : ac_rms_db;

    // A valid microphone must:
    // 1. Have active variance (min != max)
    // 2. Not be completely silent/floating (AC RMS > POST_MIC_FLATLINE_RMS_DB)
    // 3. Not be pegged against the rail (neither AC nor DC RMS >= POST_MIC_RAIL_SHORT_RMS_DB)
    out_ok = (out_min != out_max) &&
             (ac_rms_db > POST_MIC_FLATLINE_RMS_DB) &&
             (raw_rms_db < POST_MIC_RAIL_SHORT_RMS_DB) &&
             (ac_rms_db < POST_MIC_RAIL_SHORT_RMS_DB);
}

static size_t count_identical_samples(const int32_t* a, const int32_t* b, size_t count) {
    size_t identical = 0;
    for (size_t i = 0; i < count; ++i) {
        if (a[i] == b[i]) {
            identical++;
        }
    }
    return identical;
}

static bool is_pair_distinct(size_t identical_count, size_t total_count) {
    return (static_cast<uint64_t>(identical_count) * 100ULL <
            static_cast<uint64_t>(total_count) * static_cast<uint64_t>(POST_MIC_MAX_COLLISION_PCT));
}

MicrophoneHealthResult evaluate_microphone_health(const int32_t* left,
                                                 const int32_t* right,
                                                 size_t count) {
    MicrophoneHealthResult result;
    init_health_result(result);

    if (left == nullptr || right == nullptr || count == 0) {
        return result;
    }

    evaluate_single_channel(left, count, result.left_min, result.left_max,
                            result.left_rms_db, result.left_ok);
    evaluate_single_channel(right, count, result.right_min, result.right_max,
                            result.right_rms_db, result.right_ok);

    result.identical_samples = count_identical_samples(left, right, count);
    result.stereo_distinct = is_pair_distinct(result.identical_samples, count);

    // Mirror onto Bus 0 channels for unified struct inspection
    result.ch_min[0] = result.left_min;
    result.ch_max[0] = result.left_max;
    result.ch_rms_db[0] = result.left_rms_db;
    result.ch_ok[0] = result.left_ok;

    result.ch_min[1] = result.right_min;
    result.ch_max[1] = result.right_max;
    result.ch_rms_db[1] = result.right_rms_db;
    result.ch_ok[1] = result.right_ok;

    result.bus0_distinct = result.stereo_distinct;
    result.all_ok = result.left_ok && result.right_ok && result.stereo_distinct;

    return result;
}

MicrophoneHealthResult evaluate_quad_microphone_health(
    const int32_t* const channels[AUDIO_NUM_CHANNELS],
    size_t count) {
    MicrophoneHealthResult result;
    init_health_result(result);

    if (channels == nullptr || count == 0) {
        return result;
    }
    for (size_t c = 0; c < AUDIO_NUM_CHANNELS; ++c) {
        if (channels[c] == nullptr) {
            return result;
        }
    }

    for (size_t c = 0; c < AUDIO_NUM_CHANNELS; ++c) {
        evaluate_single_channel(channels[c], count,
                                result.ch_min[c], result.ch_max[c],
                                result.ch_rms_db[c], result.ch_ok[c]);
    }

    // Intra-bus collision checks: Bus 0 (ch0 vs ch1) and Bus 1 (ch2 vs ch3)
    const size_t ident_01 = count_identical_samples(channels[0], channels[1], count);
    const size_t ident_23 = count_identical_samples(channels[2], channels[3], count);
    result.bus0_distinct = is_pair_distinct(ident_01, count);
    result.bus1_distinct = is_pair_distinct(ident_23, count);

    // Cross-bus collision checks: verify no channel on Bus 0 is shorted to Bus 1
    const size_t ident_02 = count_identical_samples(channels[0], channels[2], count);
    const size_t ident_03 = count_identical_samples(channels[0], channels[3], count);
    const size_t ident_12 = count_identical_samples(channels[1], channels[2], count);
    const size_t ident_13 = count_identical_samples(channels[1], channels[3], count);
    result.cross_bus_distinct = is_pair_distinct(ident_02, count) &&
                                is_pair_distinct(ident_03, count) &&
                                is_pair_distinct(ident_12, count) &&
                                is_pair_distinct(ident_13, count);

    // Populate legacy 2-channel fields from Bus 0 and overall distinctness
    result.left_min = result.ch_min[0];
    result.left_max = result.ch_max[0];
    result.left_rms_db = result.ch_rms_db[0];
    result.left_ok = result.ch_ok[0];

    result.right_min = result.ch_min[1];
    result.right_max = result.ch_max[1];
    result.right_rms_db = result.ch_rms_db[1];
    result.right_ok = result.ch_ok[1];

    size_t max_ident = ident_01;
    if (ident_23 > max_ident) max_ident = ident_23;
    if (ident_02 > max_ident) max_ident = ident_02;
    if (ident_03 > max_ident) max_ident = ident_03;
    if (ident_12 > max_ident) max_ident = ident_12;
    if (ident_13 > max_ident) max_ident = ident_13;

    result.identical_samples = max_ident;
    result.stereo_distinct = result.bus0_distinct &&
                             result.bus1_distinct &&
                             result.cross_bus_distinct;

    result.all_ok = result.ch_ok[0] && result.ch_ok[1] &&
                    result.ch_ok[2] && result.ch_ok[3] &&
                    result.bus0_distinct && result.bus1_distinct &&
                    result.cross_bus_distinct;

    return result;
}

MicrophoneHealthResult evaluate_quad_microphone_health(
    const int32_t* ch0,
    const int32_t* ch1,
    const int32_t* ch2,
    const int32_t* ch3,
    size_t count) {
    const int32_t* const channels[AUDIO_NUM_CHANNELS] = {ch0, ch1, ch2, ch3};
    return evaluate_quad_microphone_health(channels, count);
}
