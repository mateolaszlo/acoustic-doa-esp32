#include "gcc_phat.h"
#include <esp_log.h>

static const char* TAG = "GCC_PHAT";

GccPhat::GccPhat(float sample_rate, float mic_distance_m, float temp_c)
    : fs(sample_rate), d(mic_distance_m), correlation_threshold(0.35f), db_threshold(-45.0f) {
    c_sound = calculateSoundSpeed(temp_c);
}

float GccPhat::calculateSoundSpeed(float temp_c) {
    return 331.3f * sqrtf(1.0f + (temp_c / 273.15f));
}

void GccPhat::updateCalibration(float distance_m, float temp_c, float corr_thresh, float db_thresh) {
    this->d = distance_m;
    this->c_sound = calculateSoundSpeed(temp_c);
    this->correlation_threshold = corr_thresh;
    this->db_threshold = db_thresh;
}

bool GccPhat::init() {
    // Inicializacija FFT tabel v ESP-DSP (vektorska pospešitev)
    esp_err_t ret = dsps_fft2r_init_fc32(NULL, N_SAMPLES);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Napaka pri inicializaciji FFT2R: %d", ret);
        return false;
    }

    // Priprava Hannovega okna
    dsps_wind_hann_f32(window, N_SAMPLES);
    return true;
}

DoaResult GccPhat::process(const int32_t* left_channel, const int32_t* right_channel) {
    DoaResult result = {0.0f, 0.0f, 0.0f, -100.0f, false};
    float energy_sum = 0.0f;

    // 1. Normalizacija 24/32-bit I2S podatkov na float [-1.0, 1.0] in oknenje
    for (int i = 0; i < N_SAMPLES; i++) {
        float s1 = ((float)left_channel[i] / 2147483648.0f) * window[i];
        float s2 = ((float)right_channel[i] / 2147483648.0f) * window[i];

        energy_sum += s1 * s1;

        // Komponente v kompleksni obliki [Real, Imag]
        fft_x1[2 * i]     = s1;
        fft_x1[2 * i + 1] = 0.0f;
        fft_x2[2 * i]     = s2;
        fft_x2[2 * i + 1] = 0.0f;
    }

    // Izračun ravni glasnosti (dBFS)
    float rms = sqrtf(energy_sum / N_SAMPLES);
    result.rms_db = (rms > 1e-5f) ? 20.0f * log10f(rms) : -100.0f;

    // Če je signal pretišji, preskočimo zahtevno računanje
    if (result.rms_db < db_threshold) {
        return result;
    }

    // 2. Izvedba FFT za oba kanala
    dsps_fft2r_fc32(fft_x1, N_SAMPLES);
    dsps_fft2r_fc32(fft_x2, N_SAMPLES);
    dsps_bit_rev_fc32(fft_x1, N_SAMPLES);
    dsps_bit_rev_fc32(fft_x2, N_SAMPLES);

    // 3. Navzkrižni spekter moči z GCC-PHAT normalizacijo
    // G12(f) = X1(f) * conj(X2(f))
    // Psi(f) = G12(f) / |G12(f)|
    for (int i = 0; i < N_SAMPLES; i++) {
        float re1 = fft_x1[2 * i];
        float im1 = fft_x1[2 * i + 1];
        float re2 = fft_x2[2 * i];
        float im2 = fft_x2[2 * i + 1];

        // Množenje s konjugirano vrednostjo: (re1 + j*im1) * (re2 - j*im2)
        float cross_re = re1 * re2 + im1 * im2;
        float cross_im = im1 * re2 - re1 * im2;

        float magnitude = sqrtf(cross_re * cross_re + cross_im * cross_im);

        if (magnitude > 1e-8f) {
            ccorr[2 * i]     = cross_re / magnitude;
            ccorr[2 * i + 1] = cross_im / magnitude;
        } else {
            ccorr[2 * i]     = 0.0f;
            ccorr[2 * i + 1] = 0.0f;
        }
    }

    // 4. Inverzni FFT (IFFT)
    // Ker v esp-dsp IFFT izračunamo preko zamenjave imag delov ali konjugacije:
    for (int i = 0; i < N_SAMPLES; i++) {
        ccorr[2 * i + 1] = -ccorr[2 * i + 1]; // Konjugiramo pred FFT
    }
    dsps_fft2r_fc32(ccorr, N_SAMPLES);
    dsps_bit_rev_fc32(ccorr, N_SAMPLES);

    // 5. Iskanje vrha korelacije znotraj fizično mogočih zakasnitev
    // Maksimalna možna zakasnitev (zvok prepotuje razdaljo d): tau_max = (d / c) * fs
    int max_lag = (int)ceilf((d / c_sound) * fs) + 1;
    
    int best_idx = 0;
    float best_peak = -1.0f;

    for (int lag = -max_lag; lag <= max_lag; lag++) {
        int idx = (lag < 0) ? (N_SAMPLES + lag) : lag;
        float val = ccorr[2 * idx] / N_SAMPLES; // Realni del po IFFT

        if (val > best_peak) {
            best_peak = val;
            best_idx = lag;
        }
    }

    result.confidence = best_peak;

    if (best_peak < correlation_threshold) {
        return result; // Ni dovolj jasnega vrha (odmev ali šum)
    }

    // 6. Podvzorčna parabolična interpolacija (za višjo natančnost od 1 vzorca)
    int idx_m1 = (best_idx - 1 < 0) ? (N_SAMPLES + best_idx - 1) : (best_idx - 1);
    int idx_p1 = (best_idx + 1 < 0) ? (N_SAMPLES + best_idx + 1) : (best_idx + 1);

    float alpha = ccorr[2 * idx_m1] / N_SAMPLES;
    float beta  = best_peak;
    float gamma = ccorr[2 * idx_p1] / N_SAMPLES;

    float p = 0.0f;
    float denom = (alpha - 2.0f * beta + gamma);
    if (fabsf(denom) > 1e-6f) {
        p = 0.5f * (alpha - gamma) / denom;
    }

    float delta_samples = (float)best_idx + p;
    float delta_t = delta_samples / fs; // Zakasnitev v sekundah

    // 7. Izračun vpadnega kota: theta = arcsin((c * delta_t) / d)
    float ratio = (c_sound * delta_t) / d;

    // Zaščita pred numeričnimi napakami zunaj [-1, 1]
    if (ratio > 1.0f)  ratio = 1.0f;
    if (ratio < -1.0f) ratio = -1.0f;

    result.angle_deg = asinf(ratio) * (180.0f / M_PI);
    result.delay_us = delta_t * 1e6f;
    result.valid = true;

    return result;
}