#include "ui/display_filter.h"
#include <cmath>

constexpr float DisplaySmoothingFilter::DEFAULT_EMA_ALPHA;
constexpr float DisplaySmoothingFilter::DEFAULT_OUTLIER_THRESHOLD_DEG;
constexpr uint8_t DisplaySmoothingFilter::DEFAULT_OUTLIER_CONFIRM_COUNT;
constexpr uint32_t DisplaySmoothingFilter::DEFAULT_RESET_SILENCE_MS;

DisplaySmoothingFilter::DisplaySmoothingFilter(float alpha,
                                               float outlier_thresh_deg,
                                               uint8_t outlier_confirm_count,
                                               uint32_t reset_silence_ms)
    : ema_alpha(alpha),
      outlier_threshold_deg(outlier_thresh_deg),
      outlier_confirm_count(outlier_confirm_count),
      reset_silence_ms(reset_silence_ms),
      smoothed_angle_deg(0.0f),
      candidate_outlier_angle_deg(0.0f),
      last_event_time_ms(0),
      consecutive_outliers(0),
      has_smoothed_angle(false) {}

float DisplaySmoothingFilter::wrapAngle180(float deg) {
    return wrap_angle_180(deg);
}

float DisplaySmoothingFilter::shortestAngularDiff(float target, float current) {
    if (!std::isfinite(target) || !std::isfinite(current)) {
        return 0.0f;
    }
    return shortest_angular_diff(target, current);
}

float DisplaySmoothingFilter::update(float raw_angle, uint32_t now_ms) {
    if (!std::isfinite(raw_angle)) {
        return smoothed_angle_deg;
    }
    const float wrapped_raw = wrapAngle180(raw_angle);

    // Reset smoothing after silence (> 1500 ms) or on first reading
    if (!has_smoothed_angle || (now_ms - last_event_time_ms > reset_silence_ms)) {
        smoothed_angle_deg = wrapped_raw;
        candidate_outlier_angle_deg = wrapped_raw;
        has_smoothed_angle = true;
        consecutive_outliers = 0;
    } else {
        const float diff = shortestAngularDiff(wrapped_raw, smoothed_angle_deg);
        if (std::fabs(diff) > outlier_threshold_deg) {
            // Check if this outlier agrees with the previous candidate outlier on S^1
            const float candidate_diff =
                shortestAngularDiff(wrapped_raw, candidate_outlier_angle_deg);
            if (consecutive_outliers > 0 &&
                std::fabs(candidate_diff) <= outlier_threshold_deg) {
                consecutive_outliers++;
                if (consecutive_outliers >= outlier_confirm_count) {
                    // Confirmed sound source relocation
                    smoothed_angle_deg = wrapped_raw;
                    candidate_outlier_angle_deg = wrapped_raw;
                    consecutive_outliers = 0;
                }
            } else {
                // First outlier, or outlier inconsistent with previous candidate
                candidate_outlier_angle_deg = wrapped_raw;
                consecutive_outliers = 1;
            }
            // Isolated or conflicting outlier is suppressed for visual display needle
        } else {
            consecutive_outliers = 0;
            smoothed_angle_deg = wrapAngle180(smoothed_angle_deg + ema_alpha * diff);
        }
    }

    last_event_time_ms = now_ms;
    return smoothed_angle_deg;
}

float DisplaySmoothingFilter::getSmoothedAngle() const {
    return smoothed_angle_deg;
}

bool DisplaySmoothingFilter::isInitialized() const {
    return has_smoothed_angle;
}

uint8_t DisplaySmoothingFilter::getConsecutiveOutliers() const {
    return consecutive_outliers;
}

void DisplaySmoothingFilter::reset() {
    has_smoothed_angle = false;
    smoothed_angle_deg = 0.0f;
    candidate_outlier_angle_deg = 0.0f;
    consecutive_outliers = 0;
    last_event_time_ms = 0;
}

MultiTargetDisplayTracker::MultiTargetDisplayTracker() {
    reset();
}

void MultiTargetDisplayTracker::reset() {
    for (size_t i = 0; i < DOA_MAX_SOURCES; ++i) {
        filters[i].reset();
        last_confidence[i] = 0.0f;
        last_update_ms[i] = 0;
        track_active[i] = false;
    }
}

void MultiTargetDisplayTracker::update(const MultiDoaResult& multi_res, uint32_t now_ms) {
    if (!multi_res.valid || multi_res.count == 0) {
        return;
    }
    float angles[DOA_MAX_SOURCES];
    float confs[DOA_MAX_SOURCES];
    const size_t cnt = (multi_res.count > DOA_MAX_SOURCES) ? DOA_MAX_SOURCES : multi_res.count;
    for (size_t s = 0; s < cnt; ++s) {
        angles[s] = multi_res.sources[s].angle_deg;
        confs[s] = multi_res.sources[s].confidence;
    }
    update(angles, confs, cnt, now_ms);
}

void MultiTargetDisplayTracker::prune(uint32_t now_ms) {
    constexpr uint32_t track_timeout_ms = DisplaySmoothingFilter::DEFAULT_RESET_SILENCE_MS;
    for (size_t i = 0; i < DOA_MAX_SOURCES; ++i) {
        if (track_active[i] && (now_ms - last_update_ms[i] > track_timeout_ms)) {
            track_active[i] = false;
            filters[i].reset();
        }
    }
}

void MultiTargetDisplayTracker::update(const float* raw_angles, const float* confidences,
                                      size_t count, uint32_t now_ms) {
    if (raw_angles == nullptr || confidences == nullptr || count == 0) {
        return;
    }

    // 1. Preveri potek neaktivnih sledi
    prune(now_ms);

    const size_t num_dets = (count > DOA_MAX_SOURCES) ? DOA_MAX_SOURCES : count;
    bool det_assigned[DOA_MAX_SOURCES] = {false};
    bool track_assigned[DOA_MAX_SOURCES] = {false};

    // 2. Požrešno prirejanje najbližjega soseda znotraj kota 45 stopinj
    constexpr float association_gate_deg = 45.0f;
    for (size_t pass = 0; pass < num_dets; ++pass) {
        float min_dist = 1e9f;
        int best_det = -1;
        int best_track = -1;

        for (size_t d = 0; d < num_dets; ++d) {
            if (det_assigned[d]) continue;
            for (size_t t = 0; t < DOA_MAX_SOURCES; ++t) {
                if (!track_active[t] || track_assigned[t]) continue;
                const float dist = std::fabs(
                    DisplaySmoothingFilter::shortestAngularDiff(raw_angles[d], filters[t].getSmoothedAngle()));
                if (dist < min_dist) {
                    min_dist = dist;
                    best_det = static_cast<int>(d);
                    best_track = static_cast<int>(t);
                }
            }
        }

        if (best_det >= 0 && best_track >= 0 && min_dist <= association_gate_deg) {
            filters[best_track].update(raw_angles[best_det], now_ms);
            last_confidence[best_track] = confidences[best_det];
            last_update_ms[best_track] = now_ms;
            det_assigned[best_det] = true;
            track_assigned[best_track] = true;
        } else {
            break;
        }
    }

    // 3. Za neprirejene zaznave poišči prosto sled ali inicializiraj novo
    for (size_t d = 0; d < num_dets; ++d) {
        if (det_assigned[d]) continue;

        for (size_t t = 0; t < DOA_MAX_SOURCES; ++t) {
            if (!track_active[t]) {
                filters[t].reset();
                filters[t].update(raw_angles[d], now_ms);
                last_confidence[t] = confidences[d];
                last_update_ms[t] = now_ms;
                track_active[t] = true;
                det_assigned[d] = true;
                break;
            }
        }
    }
}

size_t MultiTargetDisplayTracker::getActiveTracks(TrackedTarget* out_targets, size_t max_targets, uint32_t now_ms) {
    if (now_ms > 0) {
        prune(now_ms);
    }
    return static_cast<const MultiTargetDisplayTracker*>(this)->getActiveTracks(out_targets, max_targets);
}

size_t MultiTargetDisplayTracker::getActiveTracks(TrackedTarget* out_targets, size_t max_targets) const {
    if (out_targets == nullptr || max_targets == 0) {
        return 0;
    }

    size_t count = 0;
    for (size_t i = 0; i < DOA_MAX_SOURCES && count < max_targets; ++i) {
        if (track_active[i]) {
            out_targets[count].smoothed_angle_deg = filters[i].getSmoothedAngle();
            out_targets[count].confidence = last_confidence[i];
            out_targets[count].active = true;
            out_targets[count].slot_id = static_cast<uint8_t>(i);
            count++;
        }
    }

    return count;
}
