#include "gcc_phat.h"
#include <esp_log.h>
#include <esp_heap_caps.h>
#include <climits>

static const char* TAG = "GCC_PHAT";

// Noise Floor Tracker Leaky Integrator Slew Constants
static constexpr float NOISE_FLOOR_ALPHA_DOWN  = 0.02f;   // Fast tracking downward (~1-2 s)
static constexpr float NOISE_FLOOR_ALPHA_UP    = 0.005f;  // Slow tracking upward under trigger
static constexpr float NOISE_FLOOR_ALPHA_EVENT = 0.0005f; // Creep tracking during acoustic events
static constexpr float NOISE_FLOOR_MIN_DB      = -85.0f;
static constexpr float NOISE_FLOOR_MAX_DB      = -20.0f;
static constexpr float NOISE_FLOOR_INITIAL_DB  = -100.0f;

namespace {

struct MicPairDescriptor {
    uint8_t mic_i;
    uint8_t mic_j;
    bool is_diagonal;
    float dir_x;
    float dir_y;
    uint8_t tri_a;
    uint8_t tri_b;
};

constexpr MicPairDescriptor MIC_PAIRS[AUDIO_NUM_PAIRS] = {
    {0, 1, false,  0.0f,  1.0f, 0, 2}, // Pair 0: Right Edge (+Y), 300-2200 Hz
    {1, 2, false, -1.0f,  0.0f, 0, 3}, // Pair 1: Front Edge (-X), 300-2200 Hz
    {3, 2, false,  0.0f,  1.0f, 1, 3}, // Pair 2: Left Edge (+Y), 300-2200 Hz
    {0, 3, false, -1.0f,  0.0f, 1, 2}, // Pair 3: Back Edge (-X), 300-2200 Hz
    {3, 1, true,   1.0f,  1.0f, 2, 3}, // Pair 4: Diagonal LB -> RF (+X, +Y), 300-1450 Hz
    {0, 2, true,  -1.0f,  1.0f, 0, 1}  // Pair 5: Diagonal RB -> LF (-X, +Y), 300-1450 Hz
};

float wrap_angle_180(float angle_deg) {
    if (!std::isfinite(angle_deg)) {
        return 0.0f;
    }
    float wrapped = fmodf(angle_deg + 180.0f, 360.0f);
    if (wrapped < 0.0f) {
        wrapped += 360.0f;
    }
    wrapped -= 180.0f;
    if (wrapped == -180.0f && angle_deg > 0.0f) {
        return 180.0f;
    }
    return wrapped;
}

} // namespace

float GccPhat::computeBandWeight(float frequency_hz, float min_hz,
                                 float max_hz, float ramp_hz) {
    if (!std::isfinite(frequency_hz) || max_hz < min_hz || ramp_hz <= 0.0f) {
        return 0.0f;
    }

    const float lower_ramp_start = min_hz - ramp_hz;
    const float upper_ramp_end = max_hz + ramp_hz;

    if (frequency_hz <= lower_ramp_start ||
        frequency_hz >= upper_ramp_end) {
        return 0.0f;
    }
    if (frequency_hz < min_hz) {
        const float progress = (frequency_hz - lower_ramp_start) / ramp_hz;
        return 0.5f * (1.0f - cosf(static_cast<float>(M_PI) * progress));
    }
    if (frequency_hz <= max_hz) {
        return 1.0f;
    }

    const float progress = (upper_ramp_end - frequency_hz) / ramp_hz;
    return 0.5f * (1.0f - cosf(static_cast<float>(M_PI) * progress));
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
    : fs(sample_rate), d(mic_distance_m),
      correlation_threshold(DEFAULT_CORRELATION_THRESHOLD),
      db_threshold(-45.0f), angle_offset_deg(DEFAULT_ANGLE_OFFSET_DEG), fft_size(0),
      adaptive_threshold_enabled(false), snr_margin_db(DEFAULT_SNR_MARGIN_DB),
      noise_floor_db(NOISE_FLOOR_INITIAL_DB), noise_floor_initialized(false),
      window(nullptr), band_weight_edge(nullptr), band_weight_diag(nullptr),
      fft_ch{nullptr, nullptr, nullptr, nullptr}, ccorr(nullptr),
      weight_sum_edge(0.0f), weight_sum_diag(0.0f),
      last_pair_lags_samples{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
      last_pair_confidences{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
      last_pair_psi{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
      last_pair_weights{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f} {
    c_sound = calculateSoundSpeed(temp_c, humidity_pct, pressure_pa);
}

GccPhat::GccPhat(float sample_rate, float mic_distance_m, float temp_c,
                 size_t window_size, float humidity_pct, float pressure_pa)
    : fs(sample_rate), d(mic_distance_m),
      correlation_threshold(DEFAULT_CORRELATION_THRESHOLD),
      db_threshold(-45.0f), angle_offset_deg(DEFAULT_ANGLE_OFFSET_DEG), fft_size(0),
      adaptive_threshold_enabled(false), snr_margin_db(DEFAULT_SNR_MARGIN_DB),
      noise_floor_db(NOISE_FLOOR_INITIAL_DB), noise_floor_initialized(false),
      window(nullptr), band_weight_edge(nullptr), band_weight_diag(nullptr),
      fft_ch{nullptr, nullptr, nullptr, nullptr}, ccorr(nullptr),
      weight_sum_edge(0.0f), weight_sum_diag(0.0f),
      last_pair_lags_samples{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
      last_pair_confidences{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
      last_pair_psi{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
      last_pair_weights{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f} {
    c_sound = calculateSoundSpeed(temp_c, humidity_pct, pressure_pa);
    fft_size = window_size;
}

GccPhat::~GccPhat() {
    releaseBuffers();
}

float GccPhat::calculateSoundSpeed(float temp_c, float humidity_pct, float pressure_pa) {
    // Zaščita pred neveljavnimi odčitki senzorja
    if (!std::isfinite(temp_c) || temp_c < -40.0f || temp_c > 85.0f) {
        temp_c = DEFAULT_TEMPERATURE_C;
    }
    if (!std::isfinite(humidity_pct) || humidity_pct < 0.0f || humidity_pct > 100.0f) {
        humidity_pct = DEFAULT_HUMIDITY_PCT;
    }
    if (!std::isfinite(pressure_pa) || pressure_pa < 30000.0f || pressure_pa > 120000.0f) {
        pressure_pa = DEFAULT_PRESSURE_PA;
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
                               float pressure_pa, float corr_thresh, float db_thresh,
                               bool adaptive_thresh, float snr_margin, float angle_offset) {
    this->d = distance_m;
    this->c_sound = calculateSoundSpeed(temp_c, humidity_pct, pressure_pa);
    this->correlation_threshold = corr_thresh;
    this->db_threshold = db_thresh;
    this->adaptive_threshold_enabled = adaptive_thresh;
    if (std::isfinite(snr_margin) && snr_margin >= 0.0f) {
        this->snr_margin_db = snr_margin;
    }
    if (std::isfinite(angle_offset) && angle_offset >= -80.0f && angle_offset <= 80.0f) {
        this->angle_offset_deg = angle_offset;
    }
}

void GccPhat::updateCalibration(float distance_m, float temp_c, float corr_thresh, float db_thresh,
                               bool adaptive_thresh, float snr_margin, float angle_offset) {
    updateCalibration(distance_m, temp_c, 50.0f, 101325.0f, corr_thresh, db_thresh,
                      adaptive_thresh, snr_margin, angle_offset);
}

void GccPhat::setAngleOffsetDeg(float offset_deg) {
    if (std::isfinite(offset_deg) && offset_deg >= -80.0f && offset_deg <= 80.0f) {
        this->angle_offset_deg = offset_deg;
    }
}

void GccPhat::setAdaptiveThreshold(bool enabled, float snr_margin) {
    this->adaptive_threshold_enabled = enabled;
    if (std::isfinite(snr_margin) && snr_margin >= 0.0f) {
        this->snr_margin_db = snr_margin;
    }
}

float GccPhat::getEffectiveThresholdDb() const {
    if (adaptive_threshold_enabled && noise_floor_initialized) {
        return noise_floor_db + snr_margin_db;
    }
    const float fixed_dbfs = (db_threshold < 0.0f)
                                 ? db_threshold
                                 : (db_threshold - DEFAULT_DB_SPL_OFFSET);
    return (fixed_dbfs > NOISE_FLOOR_MAX_DB) ? NOISE_FLOOR_MAX_DB : fixed_dbfs;
}

void GccPhat::resetNoiseFloor(float initial_db) {
    noise_floor_db = initial_db;
    noise_floor_initialized = (initial_db >= NOISE_FLOOR_MIN_DB);
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

    const size_t real_bytes = window_size * sizeof(float);
    const size_t complex_bytes = window_size * 2 * sizeof(float);

    float* new_window = static_cast<float*>(
        heap_caps_malloc(real_bytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
    float* new_band_weight_edge = static_cast<float*>(
        heap_caps_malloc(real_bytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
    float* new_band_weight_diag = static_cast<float*>(
        heap_caps_malloc(real_bytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
    float* new_fft_ch[AUDIO_NUM_CHANNELS] = {nullptr, nullptr, nullptr, nullptr};
    for (size_t c = 0; c < AUDIO_NUM_CHANNELS; ++c) {
        new_fft_ch[c] = static_cast<float*>(
            heap_caps_malloc(complex_bytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
    }
    float* new_ccorr = static_cast<float*>(
        heap_caps_malloc(complex_bytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));

    // Large windows may not fit in internal RAM. Keep the preferred allocation
    // above, but use PSRAM for any buffer that could not be allocated there.
    auto allocate_psram = [](float* current, size_t bytes) -> float* {
        return current != nullptr
            ? current
            : static_cast<float*>(heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    };
    new_window = allocate_psram(new_window, real_bytes);
    new_band_weight_edge = allocate_psram(new_band_weight_edge, real_bytes);
    new_band_weight_diag = allocate_psram(new_band_weight_diag, real_bytes);
    for (size_t c = 0; c < AUDIO_NUM_CHANNELS; ++c) {
        new_fft_ch[c] = allocate_psram(new_fft_ch[c], complex_bytes);
    }
    new_ccorr = allocate_psram(new_ccorr, complex_bytes);

    auto free_new_buffers = [&]() {
        heap_caps_free(new_window);
        heap_caps_free(new_band_weight_edge);
        heap_caps_free(new_band_weight_diag);
        for (size_t c = 0; c < AUDIO_NUM_CHANNELS; ++c) {
            heap_caps_free(new_fft_ch[c]);
        }
        heap_caps_free(new_ccorr);
    };

    if (new_window == nullptr || new_band_weight_edge == nullptr ||
        new_band_weight_diag == nullptr || new_fft_ch[0] == nullptr ||
        new_fft_ch[1] == nullptr || new_fft_ch[2] == nullptr ||
        new_fft_ch[3] == nullptr || new_ccorr == nullptr) {
        free_new_buffers();
        ESP_LOGE(TAG, "Pomnilnika za FFT velikost %u ni mogoče rezervirati",
                 static_cast<unsigned>(window_size));
        return false;
    }

    // Inicializacija FFT tabel v ESP-DSP (vektorska pospešitev)
    esp_err_t ret = dsps_fft2r_init_fc32(NULL, static_cast<int>(window_size));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Napaka pri inicializaciji FFT2R: %d", ret);
        free_new_buffers();
        return false;
    }

    releaseBuffers();
    window = new_window;
    band_weight_edge = new_band_weight_edge;
    band_weight_diag = new_band_weight_diag;
    for (size_t c = 0; c < AUDIO_NUM_CHANNELS; ++c) {
        fft_ch[c] = new_fft_ch[c];
    }
    ccorr = new_ccorr;
    fs = sample_rate;
    fft_size = window_size;

    // Priprava Hannovega okna
    dsps_wind_hann_f32(window, static_cast<int>(fft_size));

    // Predizračun dvopasovnih uteži (300-2200 Hz za robne pare, 300-1450 Hz za diagonali)
    const float bin_hz = fs / static_cast<float>(fft_size);
    weight_sum_edge = 0.0f;
    weight_sum_diag = 0.0f;
    for (size_t i = 0; i < fft_size; ++i) {
        const size_t mirrored_bin = (i <= fft_size / 2) ? i : (fft_size - i);
        const float frequency_hz = static_cast<float>(mirrored_bin) * bin_hz;
        const float w_edge = computeBandWeight(
            frequency_hz, GCC_PHAT_MIN_HZ, GCC_PHAT_EDGE_MAX_HZ, GCC_PHAT_RAMP_HZ);
        const float w_diag = computeBandWeight(
            frequency_hz, GCC_PHAT_MIN_HZ, GCC_PHAT_DIAG_MAX_HZ, GCC_PHAT_RAMP_HZ);
        band_weight_edge[i] = w_edge;
        band_weight_diag[i] = w_diag;
        weight_sum_edge += w_edge;
        weight_sum_diag += w_diag;
    }

    resetNoiseFloor(-100.0f);
    return true;
}

bool GccPhat::updateNoiseFloorAndCheckGate(float rms_db) {
    float effective_thresh = getEffectiveThresholdDb();
    if (adaptive_threshold_enabled) {
        if (!noise_floor_initialized) {
            if (rms_db > NOISE_FLOOR_MIN_DB) {
                const float max_initial_seed = (effective_thresh < NOISE_FLOOR_MAX_DB)
                                                   ? effective_thresh
                                                   : NOISE_FLOOR_MAX_DB;
                noise_floor_db = (rms_db < max_initial_seed)
                                     ? rms_db
                                     : max_initial_seed;
                noise_floor_initialized = true;
            }
        } else {
            const float trigger_threshold = noise_floor_db + snr_margin_db;

            if (rms_db < noise_floor_db) {
                // Hiter padec ocene šuma ob utišanju ozadja (~1-2 s)
                noise_floor_db += NOISE_FLOOR_ALPHA_DOWN * (rms_db - noise_floor_db);
            } else if (rms_db < trigger_threshold) {
                // Počasno sledenje dvigu šuma okolice pod pragom proženja
                noise_floor_db += NOISE_FLOOR_ALPHA_UP * (rms_db - noise_floor_db);
            } else {
                // Zvočni dogodek / govor: zelo počasno prilagajanje, da trajen zvok
                // ne zviša takoj praga in ne zaduši detekcije
                noise_floor_db += NOISE_FLOOR_ALPHA_EVENT * (rms_db - noise_floor_db);
            }
        }

        if (noise_floor_initialized) {
            if (noise_floor_db < NOISE_FLOOR_MIN_DB) {
                noise_floor_db = NOISE_FLOOR_MIN_DB;
            } else if (noise_floor_db > NOISE_FLOOR_MAX_DB) {
                noise_floor_db = NOISE_FLOOR_MAX_DB;
            }
            effective_thresh = noise_floor_db + snr_margin_db;
        }
    }

    return rms_db >= effective_thresh;
}

void GccPhat::computePairCrossCorrelationIfft(size_t mic_i, size_t mic_j, const float* weights) {
    const float* spec_i = fft_ch[mic_i];
    const float* spec_j = fft_ch[mic_j];

    for (size_t i = 0; i < fft_size; ++i) {
        const float band_weight = weights[i];
        if (band_weight == 0.0f) {
            ccorr[2 * i]     = 0.0f;
            ccorr[2 * i + 1] = 0.0f;
            continue;
        }

        const float re1 = spec_i[2 * i];
        const float im1 = spec_i[2 * i + 1];
        const float re2 = spec_j[2 * i];
        const float im2 = spec_j[2 * i + 1];

        // Množenje s konjugirano vrednostjo: (re1 + j*im1) * (re2 - j*im2)
        const float cross_re = re1 * re2 + im1 * im2;
        const float cross_im = im1 * re2 - re1 * im2;

        const float magnitude = sqrtf(cross_re * cross_re + cross_im * cross_im);

        if (magnitude > 1e-8f) {
            ccorr[2 * i]     = (cross_re / magnitude) * band_weight;
            ccorr[2 * i + 1] = (cross_im / magnitude) * band_weight;
        } else {
            ccorr[2 * i]     = 0.0f;
            ccorr[2 * i + 1] = 0.0f;
        }
    }

    // Inverzni FFT (IFFT) preko konjugacije pred direktnim FFT
    for (size_t i = 0; i < fft_size; ++i) {
        ccorr[2 * i + 1] = -ccorr[2 * i + 1];
    }
    dsps_fft2r_fc32(ccorr, static_cast<int>(fft_size));
    dsps_bit_rev_fc32(ccorr, static_cast<int>(fft_size));
}

DoaResult GccPhat::process(const int32_t* left_channel, const int32_t* right_channel) {
    DoaResult result = {0.0f, 0.0f, 0.0f, -100.0f, false};
    if (left_channel == nullptr || right_channel == nullptr || fft_size == 0 ||
        window == nullptr || band_weight_edge == nullptr ||
        fft_ch[0] == nullptr || fft_ch[1] == nullptr || ccorr == nullptr) {
        return result;
    }

    int64_t left_sum = 0;
    int64_t right_sum = 0;
    for (size_t i = 0; i < fft_size; i++) {
        left_sum += static_cast<int64_t>(left_channel[i]);
        right_sum += static_cast<int64_t>(right_channel[i]);
    }
    const int64_t fft_size_i64 = static_cast<int64_t>(fft_size);
    const float inv_fft_size = 1.0f / static_cast<float>(fft_size);
    const int64_t left_mean_int = left_sum / fft_size_i64;
    const int64_t right_mean_int = right_sum / fft_size_i64;
    const float left_mean_rem =
        static_cast<float>(left_sum - left_mean_int * fft_size_i64) * inv_fft_size;
    const float right_mean_rem =
        static_cast<float>(right_sum - right_mean_int * fft_size_i64) * inv_fft_size;
    const float inv_scale = 1.0f / PCM_INT32_TO_FLOAT_SCALE;

    float energy_sum = 0.0f;

    // 1. Odstranitev DC odmika, normalizacija in Hannovo oknenje
    for (size_t i = 0; i < fft_size; i++) {
        const float raw_s1 =
            (static_cast<float>(static_cast<int64_t>(left_channel[i]) - left_mean_int) -
             left_mean_rem) * inv_scale;
        const float raw_s2 =
            (static_cast<float>(static_cast<int64_t>(right_channel[i]) - right_mean_int) -
             right_mean_rem) * inv_scale;

        // Izračun neoknjene stereo energije obeh kanalov za simetrično zaznavo
        energy_sum += 0.5f * (raw_s1 * raw_s1 + raw_s2 * raw_s2);

        const float w = window[i];
        // Komponente v kompleksni obliki [Real, Imag]
        fft_ch[0][2 * i]     = raw_s1 * w;
        fft_ch[0][2 * i + 1] = 0.0f;
        fft_ch[1][2 * i]     = raw_s2 * w;
        fft_ch[1][2 * i + 1] = 0.0f;
    }

    // Izračun ravni glasnosti (dBFS)
    float rms = sqrtf(energy_sum * inv_fft_size);
    result.rms_db = (rms > 1e-5f) ? 20.0f * log10f(rms) : NOISE_FLOOR_INITIAL_DB;

    // 1b. Določitev praga zaznave (prilagodljivi šum okolice ali fiksni prag)
    if (!updateNoiseFloorAndCheckGate(result.rms_db)) {
        return result;
    }

    // 2. Izvedba FFT za oba kanala
    dsps_fft2r_fc32(fft_ch[0], static_cast<int>(fft_size));
    dsps_fft2r_fc32(fft_ch[1], static_cast<int>(fft_size));
    dsps_bit_rev_fc32(fft_ch[0], static_cast<int>(fft_size));
    dsps_bit_rev_fc32(fft_ch[1], static_cast<int>(fft_size));

    // 3. Navzkrižni spekter moči z GCC-PHAT normalizacijo + 4. IFFT
    computePairCrossCorrelationIfft(0, 1, band_weight_edge);

    // 5. Iskanje vrha korelacije znotraj fizično mogočih zakasnitev
    // Maksimalna možna zakasnitev (zvok prepotuje razdaljo d): tau_max = (d / c) * fs
    int max_lag = (int)ceilf((d / c_sound) * fs) + 1;
    if (max_lag >= static_cast<int>(fft_size)) {
        max_lag = static_cast<int>(fft_size) - 1;
    }

    int best_idx = 0;
    float best_peak = -1.0f;
    const float norm_factor =
        (weight_sum_edge > 1e-6f) ? weight_sum_edge : static_cast<float>(fft_size);

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

    // Kompenzacija statičnega časovnega zamika med kanaloma (I2S/hardware bias):
    // delta_t_offset = (d * sin(angle_offset)) / c
    if (angle_offset_deg != 0.0f && std::isfinite(angle_offset_deg)) {
        const float offset_rad = angle_offset_deg * (static_cast<float>(M_PI) / 180.0f);
        const float delta_t_offset = (d * sinf(offset_rad)) / c_sound;
        delta_t -= delta_t_offset;
    }

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

DoaResult GccPhat::process(const int32_t* ch0, const int32_t* ch1,
                           const int32_t* ch2, const int32_t* ch3) {
    DoaResult result = {0.0f, 0.0f, 0.0f, -100.0f, false};
    for (size_t p = 0; p < AUDIO_NUM_PAIRS; ++p) {
        last_pair_lags_samples[p] = 0.0f;
        last_pair_confidences[p] = 0.0f;
        last_pair_psi[p] = 0.0f;
        last_pair_weights[p] = 0.0f;
    }
    if (ch0 == nullptr || ch1 == nullptr || ch2 == nullptr || ch3 == nullptr ||
        fft_size == 0 || window == nullptr ||
        band_weight_edge == nullptr || band_weight_diag == nullptr ||
        fft_ch[0] == nullptr || fft_ch[1] == nullptr ||
        fft_ch[2] == nullptr || fft_ch[3] == nullptr || ccorr == nullptr) {
        return result;
    }

    // 1. Odstranitev DC odmika po vseh 4 kanalih (v int64_t za 24-bitno natančnost brez plavanja)
    int64_t ch_sum[AUDIO_NUM_CHANNELS] = {0, 0, 0, 0};
    for (size_t i = 0; i < fft_size; ++i) {
        ch_sum[0] += static_cast<int64_t>(ch0[i]);
        ch_sum[1] += static_cast<int64_t>(ch1[i]);
        ch_sum[2] += static_cast<int64_t>(ch2[i]);
        ch_sum[3] += static_cast<int64_t>(ch3[i]);
    }
    const int64_t fft_size_i64 = static_cast<int64_t>(fft_size);
    const float inv_fft_size = 1.0f / static_cast<float>(fft_size);
    int64_t ch_mean_int[AUDIO_NUM_CHANNELS];
    float ch_mean_rem[AUDIO_NUM_CHANNELS];
    for (size_t c = 0; c < AUDIO_NUM_CHANNELS; ++c) {
        ch_mean_int[c] = ch_sum[c] / fft_size_i64;
        ch_mean_rem[c] =
            static_cast<float>(ch_sum[c] - ch_mean_int[c] * fft_size_i64) * inv_fft_size;
    }

    // 2. 4-kanalna vsesmerna (omnidirekcionalna) neoknjena RMS energija in Hannovo oknenje
    const float inv_scale = 1.0f / PCM_INT32_TO_FLOAT_SCALE;
    float energy_sum = 0.0f;
    for (size_t i = 0; i < fft_size; ++i) {
        const float raw_s0 =
            (static_cast<float>(static_cast<int64_t>(ch0[i]) - ch_mean_int[0]) - ch_mean_rem[0]) *
            inv_scale;
        const float raw_s1 =
            (static_cast<float>(static_cast<int64_t>(ch1[i]) - ch_mean_int[1]) - ch_mean_rem[1]) *
            inv_scale;
        const float raw_s2 =
            (static_cast<float>(static_cast<int64_t>(ch2[i]) - ch_mean_int[2]) - ch_mean_rem[2]) *
            inv_scale;
        const float raw_s3 =
            (static_cast<float>(static_cast<int64_t>(ch3[i]) - ch_mean_int[3]) - ch_mean_rem[3]) *
            inv_scale;

        energy_sum += 0.25f * (raw_s0 * raw_s0 + raw_s1 * raw_s1 +
                               raw_s2 * raw_s2 + raw_s3 * raw_s3);

        const float w = window[i];
        fft_ch[0][2 * i]     = raw_s0 * w;
        fft_ch[0][2 * i + 1] = 0.0f;
        fft_ch[1][2 * i]     = raw_s1 * w;
        fft_ch[1][2 * i + 1] = 0.0f;
        fft_ch[2][2 * i]     = raw_s2 * w;
        fft_ch[2][2 * i + 1] = 0.0f;
        fft_ch[3][2 * i]     = raw_s3 * w;
        fft_ch[3][2 * i + 1] = 0.0f;
    }

    const float rms = sqrtf(energy_sum * inv_fft_size);
    result.rms_db = (rms > 1e-5f) ? 20.0f * log10f(rms) : NOISE_FLOOR_INITIAL_DB;

    // 3. Prilagodljivi šumni prag
    if (!updateNoiseFloorAndCheckGate(result.rms_db)) {
        return result;
    }

    // 4. Štirje direktni kompleksni FFT-ji (X0..X3)
    for (size_t c = 0; c < AUDIO_NUM_CHANNELS; ++c) {
        dsps_fft2r_fc32(fft_ch[c], static_cast<int>(fft_size));
        dsps_bit_rev_fc32(fft_ch[c], static_cast<int>(fft_size));
    }

    // 5. Dvopasovno uteženi navzkrižni spektri moči, IFFT in parabolična interpolacija vrha za vseh 6 parov
    float pair_lags_samples[AUDIO_NUM_PAIRS];
    float pair_confidences[AUDIO_NUM_PAIRS];
    const int fft_size_int = static_cast<int>(fft_size);

    for (size_t p = 0; p < AUDIO_NUM_PAIRS; ++p) {
        const MicPairDescriptor& pair = MIC_PAIRS[p];
        const float* weights = pair.is_diagonal ? band_weight_diag : band_weight_edge;
        const float weight_sum = pair.is_diagonal ? weight_sum_diag : weight_sum_edge;
        const float norm_factor =
            (weight_sum > 1e-6f) ? weight_sum : static_cast<float>(fft_size);

        computePairCrossCorrelationIfft(pair.mic_i, pair.mic_j, weights);

        const float baseline_len = pair.is_diagonal ? (d * static_cast<float>(M_SQRT2)) : d;
        int max_lag = static_cast<int>(ceilf((baseline_len / c_sound) * fs)) + 1;
        if (max_lag >= fft_size_int) {
            max_lag = fft_size_int - 1;
        }

        int best_idx = 0;
        float best_peak = -1.0f;
        for (int lag = -max_lag; lag <= max_lag; ++lag) {
            const int idx = (lag < 0) ? (fft_size_int + lag) : lag;
            const float val = ccorr[2 * idx] / norm_factor;
            if (val > best_peak) {
                best_peak = val;
                best_idx = lag;
            }
        }

        // 3-točkovna parabolična interpolacija za podvzorčno zakasnitev in vrh
        const int idx_m1 = wrap_correlation_index(best_idx - 1, fft_size_int);
        const int idx_p1 = wrap_correlation_index(best_idx + 1, fft_size_int);

        const float alpha = ccorr[2 * idx_m1] / norm_factor;
        const float beta  = best_peak;
        const float gamma = ccorr[2 * idx_p1] / norm_factor;

        float p_offset = 0.0f;
        float peak_val = beta;
        const float denom = (alpha - 2.0f * beta + gamma);
        if (denom < -1e-6f) {
            p_offset = 0.5f * (alpha - gamma) / denom;
            if (p_offset > 0.5f) {
                p_offset = 0.5f;
            } else if (p_offset < -0.5f) {
                p_offset = -0.5f;
            }
            const float vertex_val = beta - 0.25f * (alpha - gamma) * p_offset;
            if (vertex_val > peak_val) {
                peak_val = vertex_val;
            }
        }

        pair_lags_samples[p] = static_cast<float>(best_idx) + p_offset;
        pair_confidences[p] = (peak_val > 1.0f) ? 1.0f : (peak_val < 0.0f ? 0.0f : peak_val);
    }

    // 6. Zaprto-zančno preverjanje trikotnikov in 2x2 WLS cenilnik azimuta
    return solveWlsAzimuth(pair_lags_samples, pair_confidences, result.rms_db);
}

DoaResult GccPhat::solveWlsAzimuth(const float pair_lags_samples[AUDIO_NUM_PAIRS],
                                   const float pair_confidences[AUDIO_NUM_PAIRS],
                                   float rms_db) {
    DoaResult result = {0.0f, 0.0f, 0.0f, rms_db, false};
    for (size_t p = 0; p < AUDIO_NUM_PAIRS; ++p) {
        last_pair_lags_samples[p] = 0.0f;
        last_pair_confidences[p] = 0.0f;
        last_pair_psi[p] = 0.0f;
        last_pair_weights[p] = 0.0f;
    }
    if (pair_lags_samples == nullptr || pair_confidences == nullptr ||
        !std::isfinite(d) || d <= 0.0f ||
        !std::isfinite(c_sound) || c_sound <= 0.0f ||
        !std::isfinite(fs) || fs <= 0.0f) {
        return result;
    }

    float peak_conf_sum = 0.0f;
    bool pair_valid[AUDIO_NUM_PAIRS];
    for (size_t p = 0; p < AUDIO_NUM_PAIRS; ++p) {
        const bool valid_inputs =
            std::isfinite(pair_lags_samples[p]) && std::isfinite(pair_confidences[p]);
        pair_valid[p] = valid_inputs;
        const float lag = valid_inputs ? pair_lags_samples[p] : 0.0f;
        float conf = valid_inputs ? pair_confidences[p] : 0.0f;
        if (conf > 1.0f) {
            conf = 1.0f;
        } else if (conf < 0.0f) {
            conf = 0.0f;
        }
        last_pair_lags_samples[p] = lag;
        last_pair_confidences[p] = conf;
        peak_conf_sum += conf;
    }
    result.confidence = peak_conf_sum / static_cast<float>(AUDIO_NUM_PAIRS);

    // 1. Štiri zaprto-zančne trikotniške napake (v vzorcih)
    const float dn0 = last_pair_lags_samples[0];
    const float dn1 = last_pair_lags_samples[1];
    const float dn2 = last_pair_lags_samples[2];
    const float dn3 = last_pair_lags_samples[3];
    const float dn4 = last_pair_lags_samples[4];
    const float dn5 = last_pair_lags_samples[5];

    const float tri_err[4] = {
        (pair_valid[0] && pair_valid[1] && pair_valid[5])
            ? fabsf(dn0 + dn1 - dn5)
            : INFINITY, // Trikotnik 0: (0-1-2) [Pari 0, 1, 5]
        (pair_valid[3] && pair_valid[2] && pair_valid[5])
            ? fabsf(dn3 + dn2 - dn5)
            : INFINITY, // Trikotnik 1: (0-3-2) [Pari 3, 2, 5]
        (pair_valid[0] && pair_valid[3] && pair_valid[4])
            ? fabsf(dn0 - dn3 - dn4)
            : INFINITY, // Trikotnik 2: (0-1-3) [Pari 0, 3, 4]
        (pair_valid[4] && pair_valid[1] && pair_valid[2])
            ? fabsf(dn4 + dn1 - dn2)
            : INFINITY  // Trikotnik 3: (3-1-2) [Pari 4, 1, 2]
    };

    // 2. Gaussove uteži konsistentnosti parov psi_p za zatiranje večpotnih odbojev
    const float sigma_loop = DOA_LOOP_CLOSURE_TOLERANCE_SAMPLES;
    const float inv_two_sigma_sq = 1.0f / (2.0f * sigma_loop * sigma_loop);
    const float min_pair_conf = 0.75f * correlation_threshold;

    for (size_t p = 0; p < AUDIO_NUM_PAIRS; ++p) {
        if (!pair_valid[p]) {
            last_pair_psi[p] = 0.0f;
            last_pair_weights[p] = 0.0f;
            continue;
        }
        const float e_a = tri_err[MIC_PAIRS[p].tri_a];
        const float e_b = tri_err[MIC_PAIRS[p].tri_b];
        const float e_pair = (e_a < e_b) ? e_a : e_b;
        const float psi = std::isfinite(e_pair) ? expf(-(e_pair * e_pair) * inv_two_sigma_sq) : 0.0f;
        last_pair_psi[p] = psi;

        const float conf = last_pair_confidences[p];
        if (conf >= min_pair_conf) {
            last_pair_weights[p] = (conf * conf) * psi;
        } else {
            last_pair_weights[p] = 0.0f;
        }
    }

    // Zavrnitev nekohorentnega odmeva/šuma: če noben trikotnik ni konsistenten (e_t <= 2.5)
    // in hkrati nimamo vsaj dveh pravokotnih robnih parov nad correlation_threshold
    const bool any_consistent_triangle =
        (tri_err[0] <= 2.5f) || (tri_err[1] <= 2.5f) ||
        (tri_err[2] <= 2.5f) || (tri_err[3] <= 2.5f);
    const bool has_y_edge = (last_pair_confidences[0] >= correlation_threshold) ||
                            (last_pair_confidences[2] >= correlation_threshold);
    const bool has_x_edge = (last_pair_confidences[1] >= correlation_threshold) ||
                            (last_pair_confidences[3] >= correlation_threshold);
    if (!any_consistent_triangle && !(has_y_edge && has_x_edge)) {
        return result;
    }

    // 3. Sestava in rešitev 2x2 WLS normalnih enačb: A * u = g
    float s_xx = 0.0f;
    float s_xy = 0.0f;
    float s_yy = 0.0f;
    float g_x = 0.0f;
    float g_y = 0.0f;
    float weight_sum = 0.0f;
    float weighted_conf_num = 0.0f;

    for (size_t p = 0; p < AUDIO_NUM_PAIRS; ++p) {
        const float w_p = last_pair_weights[p];
        if (w_p <= 0.0f) {
            continue;
        }
        const float b_x = MIC_PAIRS[p].dir_x * d;
        const float b_y = MIC_PAIRS[p].dir_y * d;
        const float tau_p = last_pair_lags_samples[p] / fs;

        s_xx += w_p * b_x * b_x;
        s_xy += w_p * b_x * b_y;
        s_yy += w_p * b_y * b_y;
        g_x  += c_sound * w_p * tau_p * b_x;
        g_y  += c_sound * w_p * tau_p * b_y;

        weight_sum += w_p;
        weighted_conf_num += w_p * last_pair_confidences[p];
    }

    if (weight_sum <= 1e-12f) {
        return result;
    }

    const float mean_conf = weighted_conf_num / weight_sum;
    result.confidence = (mean_conf > 1.0f) ? 1.0f : (mean_conf < 0.0f ? 0.0f : mean_conf);
    if (mean_conf < correlation_threshold) {
        return result;
    }

    const float lambda = 1e-6f * d * d;
    const float a_xx = s_xx + lambda;
    const float a_yy = s_yy + lambda;
    const float det = a_xx * a_yy - s_xy * s_xy;
    if (det <= 1e-20f) {
        return result;
    }

    const float u_x = (a_yy * g_x - s_xy * g_y) / det;
    const float u_y = (a_xx * g_y - s_xy * g_x) / det;
    const float u_norm = sqrtf(u_x * u_x + u_y * u_y);

    if (!std::isfinite(u_norm) || u_norm <= 0.05f || u_norm > 1.35f) {
        return result;
    }

    // 4. Polni 360-stopinjski azimut [-180, +180] in krožna kompenzacija angle_offset_deg
    const float phi_raw = atan2f(u_x, u_y) * (180.0f / static_cast<float>(M_PI));
    const float offset_deg = std::isfinite(angle_offset_deg) ? angle_offset_deg : 0.0f;
    const float phi_cal = wrap_angle_180(phi_raw - offset_deg);
    const float phi_cal_rad = phi_cal * (static_cast<float>(M_PI) / 180.0f);

    result.angle_deg = phi_cal;
    result.delay_us = ((d * sinf(phi_cal_rad)) / c_sound) * 1e6f;
    result.valid = true;
    return result;
}

void GccPhat::releaseBuffers() {
    heap_caps_free(window);
    heap_caps_free(band_weight_edge);
    heap_caps_free(band_weight_diag);
    for (size_t c = 0; c < AUDIO_NUM_CHANNELS; ++c) {
        heap_caps_free(fft_ch[c]);
        fft_ch[c] = nullptr;
    }
    heap_caps_free(ccorr);
    window = nullptr;
    band_weight_edge = nullptr;
    band_weight_diag = nullptr;
    ccorr = nullptr;
    weight_sum_edge = 0.0f;
    weight_sum_diag = 0.0f;
    fft_size = 0;
}