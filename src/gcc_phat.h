#pragma once
#include <esp_dsp.h>
#include <cmath>
#include <cstddef>

#define N_SAMPLES 1024 // Število vzorcev (potenca 2 za FFT)

struct DoaResult {
    float angle_deg;       // Kot θ v stopinjah [-90, +90]
    float delay_us;        // Zakasnitev v mikrosekundah
    float confidence;      // Normaliziran korelacijski vrh [0, 1]
    float rms_db;          // Glasnost v dBFS
    bool valid;
};

class GccPhat {
public:
    GccPhat(float sample_rate, float mic_distance_m, float temp_c = 20.0f,
            float humidity_pct = 50.0f, float pressure_pa = 101325.0f);
    GccPhat(float sample_rate, float mic_distance_m, float temp_c, size_t window_size,
            float humidity_pct = 50.0f, float pressure_pa = 101325.0f);
    ~GccPhat();

    // Initializes the default 1024-sample window (or the size supplied to the
    // size-aware constructor).
    bool init();
    // Initializes the FFT and all working buffers for a power-of-two window.
    bool init(size_t window_size);
    // Atomically switches the sample rate and FFT size after successful setup.
    bool configure(float sample_rate, size_t window_size);
    DoaResult process(const int32_t* left_channel, const int32_t* right_channel);
    void updateCalibration(float distance_m, float temp_c, float humidity_pct,
                           float pressure_pa, float corr_thresh, float db_thresh);
    void updateCalibration(float distance_m, float temp_c, float corr_thresh, float db_thresh);
    static float calculateSoundSpeed(float temp_c, float humidity_pct = 50.0f,
                                     float pressure_pa = 101325.0f);
    float getSoundSpeed() const { return c_sound; }

private:
    float fs;
    float d;
    float c_sound;
    float correlation_threshold;
    float db_threshold;
    size_t fft_size;

    // Medpomnilniki za FFT (kompleksna števila: [Re, Im, Re, Im...])
    float* window;
    float* fft_x1;
    float* fft_x2;
    float* ccorr;
    
    void releaseBuffers();
};