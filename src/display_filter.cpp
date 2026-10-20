#include "display_filter.h"
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

float DisplaySmoothingFilter::update(float raw_angle, uint32_t now_ms) {
    // Reset smoothing after silence (> 1500 ms) or on first reading
    if (!has_smoothed_angle || (now_ms - last_event_time_ms > reset_silence_ms)) {
        smoothed_angle_deg = raw_angle;
        candidate_outlier_angle_deg = raw_angle;
        has_smoothed_angle = true;
        consecutive_outliers = 0;
    } else {
        const float diff = raw_angle - smoothed_angle_deg;
        if (std::fabs(diff) > outlier_threshold_deg) {
            // Check if this outlier agrees with the previous candidate outlier
            if (consecutive_outliers > 0 &&
                std::fabs(raw_angle - candidate_outlier_angle_deg) <= outlier_threshold_deg) {
                consecutive_outliers++;
                if (consecutive_outliers >= outlier_confirm_count) {
                    // Confirmed sound source relocation
                    smoothed_angle_deg = raw_angle;
                    candidate_outlier_angle_deg = raw_angle;
                    consecutive_outliers = 0;
                }
            } else {
                // First outlier, or outlier inconsistent with previous candidate
                candidate_outlier_angle_deg = raw_angle;
                consecutive_outliers = 1;
            }
            // Isolated or conflicting outlier is suppressed for visual display needle
        } else {
            consecutive_outliers = 0;
            smoothed_angle_deg += ema_alpha * diff;
        }
    }

    last_event_time_ms = now_ms;

    // Clamp to physical radar bounds [-90, +90]
    if (smoothed_angle_deg < -90.0f) {
        smoothed_angle_deg = -90.0f;
    } else if (smoothed_angle_deg > 90.0f) {
        smoothed_angle_deg = 90.0f;
    }

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
