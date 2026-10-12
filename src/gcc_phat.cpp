#include "gcc_phat.h"
#include <esp_log.h>
#include <esp_heap_caps.h>
#include <climits>

static const char* TAG = "GCC_PHAT";

GccPhat::GccPhat(float sample_rate, float mic_distance_m, float temp_c)
    : fs(sample_rate), d(mic_distance_m), correlation_threshold(0.35f),
      db_threshold(-45.0f), fft_size(0), window(nullptr), fft_x1(nullptr),
      fft_x2(nullptr), ccorr(nullptr) {
    c_sound = calculateSoundSpeed(temp_c);
}

GccPhat::GccPhat(float sample_rate, float mic_distance_m, float temp_c, size_t window_size)
    : fs(sample_rate), d(mic_distance_m), correlation_threshold(0.35f),
      db_threshold(-45.0f), fft_size(0), window(nullptr), fft_x1(nullptr),
      fft_x2(nullptr), ccorr(nullptr) {
    c_sound = calculateSoundSpeed(temp_c);
    fft_size = window_size;
}

GccPhat::~GccPhat() {
    releaseBuffers();
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
    return init(fft_size != 0 ? fft_size : N_SAMPLES);
}

bool GccPhat::init(size_t window_size) {
    return configure(fs, window_size);
}

bool GccPhat::configure(float sample_rate, size_t window_size) {
    if (!std::isfinite(sample_rate) || sample_rate <= 0.0f ||
        window_size < 2 || window_size > static_cast<size_t>(INT_MAX) ||
        (window_size & (window_size - 1)) != 0) {
        ESP_LOGE(TAG, "Neveljavna sample rate/FFT velikost: %.3f/%u",
                 sample_rate, static_cast<unsigned>(window_size));
        return false;
    }

    const size_t complex_bytes = window_size * 2 * sizeof(float);
    float* new_window = static_cast<float*>(
        heap_caps_malloc(window_size * sizeof(float), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
    float* new_fft_x1 = static_cast<float*>(
        heap_caps_malloc(complex_bytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
    float* new_fft_x2 = static_cast<float*>(
        heap_caps_malloc(complex_bytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
    float* new_ccorr = static_cast<float*>(
        heap_caps_malloc(complex_bytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));

    // Large windows may not fit in internal RAM. Keep the preferred allocation
    // above, but use PSRAM for any buffer that could not be allocated there.
    auto allocate_psram = [](float* current, size_t bytes) -> float* {
        return current != nullptr
            ? current
            : static_cast<float*>(heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    };
    new_window = allocate_psram(new_window, window_size * sizeof(float));
    new_fft_x1 = allocate_psram(new_fft_x1, complex_bytes);
    new_fft_x2 = allocate_psram(new_fft_x2, complex_bytes);
    new_ccorr = allocate_psram(new_ccorr, complex_bytes);

    if (new_window == nullptr || new_fft_x1 == nullptr || new_fft_x2 == nullptr ||
        new_ccorr == nullptr) {
        heap_caps_free(new_window);
        heap_caps_free(new_fft_x1);
        heap_caps_free(new_fft_x2);
        heap_caps_free(new_ccorr);
        ESP_LOGE(TAG, "Pomnilnika za FFT velikost %u ni mogoče rezervirati",
                 static_cast<unsigned>(window_size));
        return false;
    }

    // Inicializacija FFT tabel v ESP-DSP (vektorska pospešitev)
    esp_err_t ret = dsps_fft2r_init_fc32(NULL, static_cast<int>(window_size));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Napaka pri inicializaciji FFT2R: %d", ret);
        heap_caps_free(new_window);
        heap_caps_free(new_fft_x1);
        heap_caps_free(new_fft_x2);
        heap_caps_free(new_ccorr);
        return false;
    }

    releaseBuffers();
    window = new_window;
    fft_x1 = new_fft_x1;
    fft_x2 = new_fft_x2;
    ccorr = new_ccorr;
    fs = sample_rate;
    fft_size = window_size;

    // Priprava Hannovega okna
    dsps_wind_hann_f32(window, static_cast<int>(fft_size));
    return true;
}

DoaResult GccPhat::process(const int32_t* left_channel, const int32_t* right_channel) {
    DoaResult result = {0.0f, 0.0f, 0.0f, -100.0f, false};
    if (left_channel == nullptr || right_channel == nullptr || fft_size == 0 ||
        window == nullptr || fft_x1 == nullptr || fft_x2 == nullptr || ccorr == nullptr) {
        return result;
    }

    float energy_sum = 0.0f;

    // 1. Normalizacija 24/32-bit I2S podatkov na float [-1.0, 1.0] in oknenje
    for (size_t i = 0; i < fft_size; i++) {
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
    float rms = sqrtf(energy_sum / static_cast<float>(fft_size));
    result.rms_db = (rms > 1e-5f) ? 20.0f * log10f(rms) : -100.0f;

    // Če je signal pretišji, preskočimo zahtevno računanje
    if (result.rms_db < db_threshold) {
        return result;
    }

    // 2. Izvedba FFT za oba kanala
    dsps_fft2r_fc32(fft_x1, static_cast<int>(fft_size));
    dsps_fft2r_fc32(fft_x2, static_cast<int>(fft_size));
    dsps_bit_rev_fc32(fft_x1, static_cast<int>(fft_size));
    dsps_bit_rev_fc32(fft_x2, static_cast<int>(fft_size));

    // 3. Navzkrižni spekter moči z GCC-PHAT normalizacijo
    // G12(f) = X1(f) * conj(X2(f))
    // Psi(f) = G12(f) / |G12(f)|
    for (size_t i = 0; i < fft_size; i++) {
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
    for (size_t i = 0; i < fft_size; i++) {
        ccorr[2 * i + 1] = -ccorr[2 * i + 1]; // Konjugiramo pred FFT
    }
    dsps_fft2r_fc32(ccorr, static_cast<int>(fft_size));
    dsps_bit_rev_fc32(ccorr, static_cast<int>(fft_size));

    // 5. Iskanje vrha korelacije znotraj fizično mogočih zakasnitev
    // Maksimalna možna zakasnitev (zvok prepotuje razdaljo d): tau_max = (d / c) * fs
    int max_lag = (int)ceilf((d / c_sound) * fs) + 1;
    if (max_lag >= static_cast<int>(fft_size)) {
        max_lag = static_cast<int>(fft_size) - 1;
    }
    
    int best_idx = 0;
    float best_peak = -1.0f;

    for (int lag = -max_lag; lag <= max_lag; lag++) {
        int idx = (lag < 0) ? (static_cast<int>(fft_size) + lag) : lag;
        float val = ccorr[2 * idx] / static_cast<float>(fft_size); // Realni del po IFFT

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
    int idx_m1 = (best_idx - 1 < 0) ? (static_cast<int>(fft_size) + best_idx - 1) : (best_idx - 1);
    int idx_p1 = (best_idx + 1 >= static_cast<int>(fft_size))
        ? 0
        : (best_idx + 1);

    float alpha = ccorr[2 * idx_m1] / static_cast<float>(fft_size);
    float beta  = best_peak;
    float gamma = ccorr[2 * idx_p1] / static_cast<float>(fft_size);

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

void GccPhat::releaseBuffers() {
    heap_caps_free(window);
    heap_caps_free(fft_x1);
    heap_caps_free(fft_x2);
    heap_caps_free(ccorr);
    window = nullptr;
    fft_x1 = nullptr;
    fft_x2 = nullptr;
    ccorr = nullptr;
    fft_size = 0;
}