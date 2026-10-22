#ifndef DISPLAY_FILTER_H
#define DISPLAY_FILTER_H

#include <cstdint>
#include "app_config.h"

/**
 * @brief Circular S^1 angle smoothing filter for the 360-degree radar needle on the ST7735 TFT.
 *
 * Implements a circular Exponential Moving Average (EMA) on [-180, +180] deg with
 * shortest-arc outlier rejection across the +/-180 deg boundary.
 * Isolated angle jumps (> 45 deg on S^1) are suppressed to avoid needle jitter from
 * multipath reflections. A sustained relocation requires 2 consecutive consistent
 * readings (within 45 deg of each other on S^1) before the display snaps to the new source angle.
 * Silence (> 1500 ms) resets the filter so the needle snaps immediately to the first
 * new acoustic event without EMA lag.
 */
class DisplaySmoothingFilter {
public:
    static constexpr float DEFAULT_EMA_ALPHA = 0.25f;
    static constexpr float DEFAULT_OUTLIER_THRESHOLD_DEG = 45.0f;
    static constexpr uint8_t DEFAULT_OUTLIER_CONFIRM_COUNT = 2;
    static constexpr uint32_t DEFAULT_RESET_SILENCE_MS = 1500;

    DisplaySmoothingFilter(float alpha = DEFAULT_EMA_ALPHA,
                           float outlier_thresh_deg = DEFAULT_OUTLIER_THRESHOLD_DEG,
                           uint8_t outlier_confirm_count = DEFAULT_OUTLIER_CONFIRM_COUNT,
                           uint32_t reset_silence_ms = DEFAULT_RESET_SILENCE_MS);

    static float wrapAngle180(float deg);
    static float shortestAngularDiff(float target, float current);

    float update(float raw_angle, uint32_t now_ms);

    float getSmoothedAngle() const;
    bool isInitialized() const;
    uint8_t getConsecutiveOutliers() const;

    void reset();

private:
    float ema_alpha;
    float outlier_threshold_deg;
    uint8_t outlier_confirm_count;
    uint32_t reset_silence_ms;

    float smoothed_angle_deg;
    float candidate_outlier_angle_deg;
    uint32_t last_event_time_ms;
    uint8_t consecutive_outliers;
    bool has_smoothed_angle;
};

inline float wrapAngle180(float deg) {
    return DisplaySmoothingFilter::wrapAngle180(deg);
}

inline float shortestAngularDiff(float target, float current) {
    return DisplaySmoothingFilter::shortestAngularDiff(target, current);
}

#endif // DISPLAY_FILTER_H
