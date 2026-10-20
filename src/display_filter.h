#ifndef DISPLAY_FILTER_H
#define DISPLAY_FILTER_H

#include <cstdint>

/**
 * @brief Angle smoothing filter for visual radar display needle on the ST7735 TFT.
 *
 * Implements an Exponential Moving Average (EMA) with outlier rejection.
 * Isolated angle jumps (> 45 deg) are suppressed to avoid needle jitter from
 * multipath reflections. A sustained relocation requires 2 consecutive consistent
 * readings before the display snaps to the new source angle.
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

#endif // DISPLAY_FILTER_H
