#include "gcc_phat.h"
#include <esp_log.h>
#include <esp_heap_caps.h>
#include <climits>

static const char* TAG = "GCC_PHAT";
static constexpr float GCC_PHAT_MIN_HZ = 300.0f;
static constexpr float GCC_PHAT_MAX_HZ = 2200.0f;
static constexpr float GCC_PHAT_RAMP_HZ = 100.0f;

static float gcc_phat_band_weight(float frequency_hz) {
    const float lower_ramp_start = GCC_PHAT_MIN_HZ - GCC_PHAT_RAMP_HZ;
    const float upper_ramp_end = GCC_PHAT_MAX_HZ + GCC_PHAT_RAMP_HZ;

    if (frequency_hz <= lower_ramp_start ||
        frequency_hz >= upper_ramp_end) {
        return 0.0f;
    }
    if (frequency_hz < GCC_PHAT_MIN_HZ) {
        const float progress =
            (frequency_hz - lower_ramp_start) / GCC_PHAT_RAMP_HZ;
        return 0.5f * (1.0f - cosf(M_PI * progress));
    }
    if (frequency_hz <= GCC_PHAT_MAX_HZ) {
        return 1.0f;
    }

    const float progress =
        (upper_ramp_end - frequency_hz) / GCC_PHAT_RAMP_HZ;
    return 0.5f * (1.0f - cosf(M_PI * progress));
}

static int wrap_correlation_index(int lag, int fft_size) {
    lag %= fft_size;
    if (lag < 0) {
        lag += fft_size;
    }
    return lag;
}

GccPhat::GccPhat(float sample_rate, float mic_distance_m, float temp_c,
                 float humidity_pct, float pressure_pa)
    : fs(sample_rate), d(mic_distance_m), correlation_threshold(0.35f),
      db_threshold(-45.0f), fft_size(0), window(nullptr), fft_x1(nullptr),
      fft_x2(nullptr), ccorr(nullptr) {
    c_sound = calculateSoundSpeed(temp_c, humidity_pct, pressure_pa);
}

GccPhat::GccPhat(float sample_rate, float mic_distance_m, float temp_c,
                 size_t window_size, float humidity_pct, float pressure_pa)
    : fs(sample_rate), d(mic_distance_m), correlation_threshold(0.35f),
      db_threshold(-45.0f), fft_size(0), window(nullptr), fft_x1(nullptr),
      fft_x2(nullptr), ccorr(nullptr) {
    c_sound = calculateSoundSpeed(temp_c, humidity_pct, pressure_pa);
    fft_size = window_size;
}

GccPhat::~GccPhat() {
    releaseBuffers();
}

float GccPhat::calculateSoundSpeed(float temp_c, float humidity_pct, float pressure_pa) {
    // Zaščita pred neveljavnimi odčitki senzorja
    if (!std::isfinite(temp_c) || temp_c < -40.0f || temp_c > 85.0f) {
        temp_c = 20.0f;
    }
    if (!std::isfinite(humidity_pct) || humidity_pct < 0.0f || humidity_pct > 100.0f) {
        humidity_pct = 50.0f;
    }
    if (!std::isfinite(pressure_pa) || pressure_pa < 30000.0f || pressure_pa > 120000.0f) {
        pressure_pa = 101325.0f;
    }

    // 1. Nasičeni parni tlak vode (Magnus-Tetens v Pa)
    float p_sat = 610.78f * expf((17.27f * temp_c) / (temp_c + 237.3f));

    // 2. Parcialni tlak vodne pare in molski ulomek x_w
    float p_vapor = (humidity_pct / 100.0f) * p_sat;
    float x_w = p_vapor / pressure_pa;
    if (x_w < 0.0f) {
        x_w = 0.0f;
    } else if (x_w > 0.2f) {
        x_w = 0.2f;
    }

    // 3. Termodinamske konstante vlažnega zraka
    constexpr float R_gas = 8.31446f; // J/(mol*K)
    float molar_mass = 0.0289654f * (1.0f - 0.3780f * x_w); // kg/mol
    float gamma = 1.4003f - 0.07f * x_w;
    float temp_k = temp_c + 273.15f;

    // 4. Hitrost zvoka c = sqrt(gamma * R * T / M)
    return sqrtf((gamma * R_gas * temp_k) / molar_mass);
}

void GccPhat::updateCalibration(float distance_m, float temp_c, float humidity_pct,
                               float pressure_pa, float corr_thresh, float db_thresh) {
    this->d = distance_m;
    this->c_sound = calculateSoundSpeed(temp_c, humidity_pct, pressure_pa);
    this->correlation_threshold = corr_thresh;
    this->db_threshold = db_thresh;
}

void GccPhat::updateCalibration(float distance_m, float temp_c, float corr_thresh, float db_thresh) {
    updateCalibration(distance_m, temp_c, 50.0f, 101325.0f, corr_thresh, db_thresh);
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

    float left_mean = 0.0f;
    float right_mean = 0.0f;
    for (size_t i = 0; i < fft_size; i++) {
        left_mean += static_cast<float>(left_channel[i]);
        right_mean += static_cast<float>(right_channel[i]);
    }
    left_mean /= static_cast<float>(fft_size);
    right_mean /= static_cast<float>(fft_size);

    float energy_sum = 0.0f;

    // 1. Odstranitev DC odmika, normalizacija in Hannovo oknenje
    for (size_t i = 0; i < fft_size; i++) {
        float s1 = ((static_cast<float>(left_channel[i]) - left_mean) /
                    2147483648.0f) * window[i];
        float s2 = ((static_cast<float>(right_channel[i]) - right_mean) /
                    2147483648.0f) * window[i];

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
    const float bin_hz = fs / static_cast<float>(fft_size);
    float weight_sum = 0.0f;

    for (size_t i = 0; i < fft_size; i++) {
        const size_t mirrored_bin = i <= fft_size / 2 ? i : fft_size - i;
        const float frequency_hz =
            static_cast<float>(mirrored_bin) * bin_hz;
        const float band_weight = gcc_phat_band_weight(frequency_hz);
        weight_sum += band_weight;
        if (band_weight == 0.0f) {
            ccorr[2 * i] = 0.0f;
            ccorr[2 * i + 1] = 0.0f;
            continue;
        }

        float re1 = fft_x1[2 * i];
        float im1 = fft_x1[2 * i + 1];
        float re2 = fft_x2[2 * i];
        float im2 = fft_x2[2 * i + 1];

        // Množenje s konjugirano vrednostjo: (re1 + j*im1) * (re2 - j*im2)
        float cross_re = re1 * re2 + im1 * im2;
        float cross_im = im1 * re2 - re1 * im2;

        float magnitude = sqrtf(cross_re * cross_re + cross_im * cross_im);

        if (magnitude > 1e-8f) {
            ccorr[2 * i]     = (cross_re / magnitude) * band_weight;
            ccorr[2 * i + 1] = (cross_im / magnitude) * band_weight;
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
    const float norm_factor =
        (weight_sum > 1e-6f) ? weight_sum : static_cast<float>(fft_size);

    for (int lag = -max_lag; lag <= max_lag; lag++) {
        int idx = (lag < 0) ? (static_cast<int>(fft_size) + lag) : lag;
        float val = ccorr[2 * idx] / norm_factor; // Realni del po IFFT normaliziran na [0, 1]

        if (val > best_peak) {
            best_peak = val;
            best_idx = lag;
        }
    }

    result.confidence = (best_peak > 1.0f) ? 1.0f : (best_peak < 0.0f ? 0.0f : best_peak);

    if (best_peak < correlation_threshold) {
        return result; // Ni dovolj jasnega vrha (odmev ali šum)
    }

    // 6. Podvzorčna parabolična interpolacija (za višjo natančnost od 1 vzorca)
    const int fft_size_int = static_cast<int>(fft_size);
    const int idx_m1 = wrap_correlation_index(best_idx - 1, fft_size_int);
    const int idx_p1 = wrap_correlation_index(best_idx + 1, fft_size_int);

    float alpha = ccorr[2 * idx_m1] / norm_factor;
    float beta  = best_peak;
    float gamma = ccorr[2 * idx_p1] / norm_factor;

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