#pragma once
#include <esp_dsp.h>
#include <cmath>

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
    GccPhat(float sample_rate, float mic_distance_m, float temp_c);
    bool init();
    DoaResult process(const int32_t* left_channel, const int32_t* right_channel);
    void updateCalibration(float distance_m, float temp_c, float corr_thresh, float db_thresh);

private:
    float fs;
    float d;
    float c_sound;
    float correlation_threshold;
    float db_threshold;

    // Medpomnilniki za FFT (kompleksna števila: [Re, Im, Re, Im...])
    float window[N_SAMPLES];
    float fft_x1[N_SAMPLES * 2];
    float fft_x2[N_SAMPLES * 2];
    float ccorr[N_SAMPLES * 2];
    
    float calculateSoundSpeed(float temp_c);
};