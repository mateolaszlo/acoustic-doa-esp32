#pragma once
#include <esp_dsp.h>
#include <cmath>
#include <cstddef>
#include "app_config.h"

#define N_SAMPLES AUDIO_DEFAULT_WINDOW_SIZE // Število vzorcev (potenca 2 za FFT)

struct DoaResult {
    float angle_deg;       // Kot θ v stopinjah [-90, +90]
    float delay_us;        // Zakasnitev v mikrosekundah
    float confidence;      // Normaliziran korelacijski vrh [0, 1]
    float rms_db;          // Glasnost v dBFS
    bool valid;
};

class GccPhat {
public:
    GccPhat(float sample_rate = DEFAULT_SAMPLE_RATE,
            float mic_distance_m = DEFAULT_MIC_DISTANCE_M,
            float temp_c = DEFAULT_TEMPERATURE_C,
            float humidity_pct = DEFAULT_HUMIDITY_PCT,
            float pressure_pa = DEFAULT_PRESSURE_PA);
    GccPhat(float sample_rate, float mic_distance_m, float temp_c, size_t window_size,
            float humidity_pct = DEFAULT_HUMIDITY_PCT,
            float pressure_pa = DEFAULT_PRESSURE_PA);
    ~GccPhat();

    // Initializes the default window (or the size supplied to the
    // size-aware constructor).
    bool init();
    // Initializes the FFT and all working buffers for a power-of-two window.
    bool init(size_t window_size);
    // Atomically switches the sample rate and FFT size after successful setup.
    bool configure(float sample_rate, size_t window_size);
    DoaResult process(const int32_t* left_channel, const int32_t* right_channel);
    void updateCalibration(float distance_m, float temp_c, float humidity_pct,
                           float pressure_pa, float corr_thresh, float db_thresh,
                           bool adaptive_thresh = false,
                           float snr_margin = DEFAULT_SNR_MARGIN_DB,
                           float angle_offset = DEFAULT_ANGLE_OFFSET_DEG);
    void updateCalibration(float distance_m, float temp_c, float corr_thresh, float db_thresh,
                           bool adaptive_thresh = false,
                           float snr_margin = DEFAULT_SNR_MARGIN_DB,
                           float angle_offset = DEFAULT_ANGLE_OFFSET_DEG);
    static float calculateSoundSpeed(float temp_c = DEFAULT_TEMPERATURE_C,
                                     float humidity_pct = DEFAULT_HUMIDITY_PCT,
                                     float pressure_pa = DEFAULT_PRESSURE_PA);
    float getSoundSpeed() const { return c_sound; }

    void setAngleOffsetDeg(float offset_deg);
    float getAngleOffsetDeg() const { return angle_offset_deg; }

    void setAdaptiveThreshold(bool enabled, float snr_margin = DEFAULT_SNR_MARGIN_DB);
    bool isAdaptiveThresholdEnabled() const { return adaptive_threshold_enabled; }
    float getSnrMarginDb() const { return snr_margin_db; }
    float getNoiseFloorDb() const { return noise_floor_db; }
    float getEffectiveThresholdDb() const;
    void resetNoiseFloor(float initial_db = -100.0f);
    bool isNoiseFloorInitialized() const { return noise_floor_initialized; }

private:
    float fs;
    float d;
    float c_sound;
    float correlation_threshold;
    float db_threshold;
    float angle_offset_deg;
    size_t fft_size;

    bool adaptive_threshold_enabled;
    float snr_margin_db;
    float noise_floor_db;
    bool noise_floor_initialized;

    // Medpomnilniki za FFT (kompleksna števila: [Re, Im, Re, Im...])
    float* window;
    float* fft_x1;
    float* fft_x2;
    float* ccorr;
    
    void releaseBuffers();
};