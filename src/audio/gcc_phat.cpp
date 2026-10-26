#include "audio/gcc_phat.h"
#include <esp_log.h>
#include <esp_heap_caps.h>
#include <climits>
#include <algorithm>

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
    {0, 1, false,  0.0f,  1.0f, 0, 2}, // Pair 0: Right Edge (+Y), 300-5000 Hz
    {1, 2, false, -1.0f,  0.0f, 0, 3}, // Pair 1: Front Edge (-X), 300-5000 Hz
    {3, 2, false,  0.0f,  1.0f, 1, 3}, // Pair 2: Left Edge (+Y), 300-5000 Hz
    {0, 3, false, -1.0f,  0.0f, 1, 2}, // Pair 3: Back Edge (-X), 300-5000 Hz
    {3, 1, true,   1.0f,  1.0f, 2, 3}, // Pair 4: Diagonal LB -> RF (+X, +Y), 300-4500 Hz
    {0, 2, true,  -1.0f,  1.0f, 0, 1}  // Pair 5: Diagonal RB -> LF (-X, +Y), 300-4500 Hz
};

static inline int clamp_corr_lag(int lag) {
    if (lag < -PAIR_CORR_RADIUS) return -PAIR_CORR_RADIUS;
    if (lag > PAIR_CORR_RADIUS) return PAIR_CORR_RADIUS;
    return lag;
}

static float estimateLocalPeakAmplitude(const float pair_corr[PAIR_CORR_SIZE],
                                        float center_float_lag,
                                        int search_radius = 2) {
    if (pair_corr == nullptr || !std::isfinite(center_float_lag)) {
        return 0.0f;
    }
    const int center_lag = clamp_corr_lag(static_cast<int>(roundf(center_float_lag)));
    int best_idx = center_lag;
    float best_val = -INFINITY;

    for (int l = center_lag - search_radius; l <= center_lag + search_radius; ++l) {
        if (l < -PAIR_CORR_RADIUS || l > PAIR_CORR_RADIUS) continue;
        const size_t k = static_cast<size_t>(l + PAIR_CORR_RADIUS);
        const float val = pair_corr[k];
        if (val > best_val) {
            best_val = val;
            best_idx = l;
        }
    }

    if (!std::isfinite(best_val)) {
        return 0.0f;
    }

    const int l_m1 = clamp_corr_lag(best_idx - 1);
    const int l_p1 = clamp_corr_lag(best_idx + 1);
    const float alpha = pair_corr[static_cast<size_t>(l_m1 + PAIR_CORR_RADIUS)];
    const float beta  = best_val;
    const float gamma = pair_corr[static_cast<size_t>(l_p1 + PAIR_CORR_RADIUS)];
    float peak_amp = beta;
    const float denom = (alpha - 2.0f * beta + gamma);
    if (denom < -1e-6f) {
        const float p_offset = 0.5f * (alpha - gamma) / denom;
        if (p_offset >= -0.5f && p_offset <= 0.5f) {
            const float vertex = beta - 0.25f * (alpha - gamma) * p_offset;
            if (vertex > peak_amp) peak_amp = vertex;
        }
    }
    if (peak_amp > 1.0f) peak_amp = 1.0f;
    if (peak_amp < 0.0f) peak_amp = 0.0f;
    return peak_amp;
}

static void subtractCleanKernel(float pair_corr[AUDIO_NUM_PAIRS][PAIR_CORR_SIZE],
                                size_t p,
                                const float* kernel,
                                float exact_lag,
                                float amp) {
    if (pair_corr == nullptr || kernel == nullptr || p >= AUDIO_NUM_PAIRS ||
        amp <= 0.05f || !std::isfinite(exact_lag) || !std::isfinite(amp)) {
        return;
    }
    const int base_lag = static_cast<int>(floorf(exact_lag));
    const float frac = exact_lag - static_cast<float>(base_lag); // in [0, 1)

    for (int delta_l = -static_cast<int>(CLEAN_KERNEL_RADIUS_SAMPLES);
         delta_l <= static_cast<int>(CLEAN_KERNEL_RADIUS_SAMPLES); ++delta_l) {
        const int target_lag = base_lag + delta_l;
        if (target_lag >= -PAIR_CORR_RADIUS && target_lag <= PAIR_CORR_RADIUS) {
            const size_t target_k = static_cast<size_t>(target_lag + PAIR_CORR_RADIUS);
            const size_t k0 = static_cast<size_t>(delta_l + static_cast<int>(CLEAN_KERNEL_RADIUS_SAMPLES));
            const float k_interp = (k0 > 0)
                ? ((1.0f - frac) * kernel[k0] + frac * kernel[k0 - 1])
                : ((1.0f - frac) * kernel[k0]);
            pair_corr[p][target_k] -= amp * k_interp;
        }
    }
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

static inline float circular_diff_deg(float a, float b) {
    float diff = fmodf(a - b + 180.0f, 360.0f);
    if (diff < 0.0f) {
        diff += 360.0f;
    }
    return diff - 180.0f;
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
    std::memset(clean_kernel_edge, 0, sizeof(clean_kernel_edge));
    std::memset(clean_kernel_diag, 0, sizeof(clean_kernel_diag));
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
    std::memset(clean_kernel_edge, 0, sizeof(clean_kernel_edge));
    std::memset(clean_kernel_diag, 0, sizeof(clean_kernel_diag));
    c_sound = calculateSoundSpeed(temp_c, humidity_pct, pressure_pa);
    fft_size = window_size;
}

GccPhat::~GccPhat() {
    releaseBuffers();
}

float GccPhat::calculateSoundSpeed(float temp_c, float humidity_pct, float pressure_pa) {
    // Guard against out-of-range or non-finite sensor readings
    if (!std::isfinite(temp_c) || temp_c < -40.0f || temp_c > 85.0f) {
        temp_c = DEFAULT_TEMPERATURE_C;
    }
    if (!std::isfinite(humidity_pct) || humidity_pct < 0.0f || humidity_pct > 100.0f) {
        humidity_pct = DEFAULT_HUMIDITY_PCT;
    }
    if (!std::isfinite(pressure_pa) || pressure_pa < 30000.0f || pressure_pa > 120000.0f) {
        pressure_pa = DEFAULT_PRESSURE_PA;
    }

    // 1. Saturation vapor pressure of water (Magnus-Tetens formula in Pa)
    float p_sat = 610.78f * expf((17.27f * temp_c) / (temp_c + 237.3f));

    // 2. Water vapor partial pressure and mole fraction x_w
    float p_vapor = (humidity_pct / 100.0f) * p_sat;
    float x_w = p_vapor / pressure_pa;
    if (x_w < 0.0f) {
        x_w = 0.0f;
    } else if (x_w > 0.2f) {
        x_w = 0.2f;
    }

    // 3. Moist-air thermodynamic constants
    constexpr float R_gas = 8.31446f; // J/(mol*K)
    float molar_mass = 0.0289654f * (1.0f - 0.3780f * x_w); // kg/mol
    float gamma = 1.4003f - 0.07f * x_w;
    float temp_k = temp_c + 273.15f;

    // 4. Speed of sound c = sqrt(gamma * R * T / M)
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
    if (std::isfinite(angle_offset) &&
        angle_offset >= ANGLE_OFFSET_MIN_DEG &&
        angle_offset <= ANGLE_OFFSET_MAX_DEG) {
        this->angle_offset_deg = angle_offset;
    }
}

void GccPhat::updateCalibration(float distance_m, float temp_c, float corr_thresh, float db_thresh,
                               bool adaptive_thresh, float snr_margin, float angle_offset) {
    updateCalibration(distance_m, temp_c, 50.0f, 101325.0f, corr_thresh, db_thresh,
                      adaptive_thresh, snr_margin, angle_offset);
}

void GccPhat::setAngleOffsetDeg(float offset_deg) {
    if (std::isfinite(offset_deg) &&
        offset_deg >= ANGLE_OFFSET_MIN_DEG &&
        offset_deg <= ANGLE_OFFSET_MAX_DEG) {
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
    return init(AUDIO_FFT_WINDOW_SIZE);
}

bool GccPhat::init(size_t window_size) {
    if (window_size != AUDIO_FFT_WINDOW_SIZE) {
        ESP_LOGE(TAG, "Unsupported FFT window size: %u (only %u is supported)",
                 static_cast<unsigned>(window_size),
                 static_cast<unsigned>(AUDIO_FFT_WINDOW_SIZE));
        return false;
    }
    return configure(fs, window_size);
}

bool GccPhat::configure(float sample_rate) {
    return configure(sample_rate, AUDIO_FFT_WINDOW_SIZE);
}

bool GccPhat::configure(float sample_rate, size_t window_size) {
    if (!std::isfinite(sample_rate) || sample_rate <= 0.0f ||
        window_size != AUDIO_FFT_WINDOW_SIZE) {
        ESP_LOGE(TAG, "Invalid sample rate or FFT window size: %.3f / %u",
                 sample_rate, static_cast<unsigned>(window_size));
        return false;
    }

    // One-time allocation of working buffers in internal SRAM on initialization
    if (window == nullptr) {
        const size_t real_bytes = AUDIO_FFT_WINDOW_SIZE * sizeof(float);
        const size_t complex_bytes = AUDIO_FFT_WINDOW_SIZE * 2 * sizeof(float);

        window = static_cast<float*>(
            heap_caps_malloc(real_bytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
        band_weight_edge = static_cast<float*>(
            heap_caps_malloc(real_bytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
        band_weight_diag = static_cast<float*>(
            heap_caps_malloc(real_bytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
        for (size_t c = 0; c < AUDIO_NUM_CHANNELS; ++c) {
            fft_ch[c] = static_cast<float*>(
                heap_caps_malloc(complex_bytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
        }
        ccorr = static_cast<float*>(
            heap_caps_malloc(complex_bytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));

        if (window == nullptr || band_weight_edge == nullptr ||
            band_weight_diag == nullptr || fft_ch[0] == nullptr ||
            fft_ch[1] == nullptr || fft_ch[2] == nullptr ||
            fft_ch[3] == nullptr || ccorr == nullptr) {
            releaseBuffers();
            ESP_LOGE(TAG, "Failed to allocate memory for FFT window size %u",
                     static_cast<unsigned>(AUDIO_FFT_WINDOW_SIZE));
            return false;
        }

        // Initialize ESP-DSP radix-2 FFT tables (SIMD vector acceleration)
        esp_err_t ret = dsps_fft2r_init_fc32(NULL, static_cast<int>(AUDIO_FFT_WINDOW_SIZE));
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to initialize FFT2R tables: %d", ret);
            releaseBuffers();
            return false;
        }

        // Generate Hann window (fixed 1024 samples)
        dsps_wind_hann_f32(window, static_cast<int>(AUDIO_FFT_WINDOW_SIZE));
        fft_size = AUDIO_FFT_WINDOW_SIZE;
    }

    fs = sample_rate;

    // Precompute dual-band spectral weights for the new sample rate in-place (zero heap allocation)
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

    precomputeCleanKernels();

    resetNoiseFloor(-100.0f);
    return true;
}

void GccPhat::precomputeCleanKernels() {
    const int fft_size_int = static_cast<int>(fft_size);
    if (ccorr == nullptr || fft_size_int <= 0 ||
        band_weight_edge == nullptr || band_weight_diag == nullptr) {
        return;
    }

    // 1. Edge pair kernel: 300 - 5000 Hz
    for (size_t i = 0; i < fft_size; ++i) {
        ccorr[2 * i]     = band_weight_edge[i];
        ccorr[2 * i + 1] = 0.0f;
    }
    for (size_t i = 0; i < fft_size; ++i) {
        ccorr[2 * i + 1] = -ccorr[2 * i + 1];
    }
    dsps_fft2r_fc32(ccorr, fft_size_int);
    dsps_bit_rev_fc32(ccorr, fft_size_int);

    const float peak_edge = (ccorr[0] > 1e-6f) ? ccorr[0] : 1.0f;
    for (int lag = -static_cast<int>(CLEAN_KERNEL_RADIUS_SAMPLES);
         lag <= static_cast<int>(CLEAN_KERNEL_RADIUS_SAMPLES); ++lag) {
        const int idx = (lag < 0) ? (fft_size_int + lag) : lag;
        const size_t k = static_cast<size_t>(lag + static_cast<int>(CLEAN_KERNEL_RADIUS_SAMPLES));
        clean_kernel_edge[k] = ccorr[2 * idx] / peak_edge;
    }

    // 2. Diagonal pair kernel: 300 - 4500 Hz
    for (size_t i = 0; i < fft_size; ++i) {
        ccorr[2 * i]     = band_weight_diag[i];
        ccorr[2 * i + 1] = 0.0f;
    }
    for (size_t i = 0; i < fft_size; ++i) {
        ccorr[2 * i + 1] = -ccorr[2 * i + 1];
    }
    dsps_fft2r_fc32(ccorr, fft_size_int);
    dsps_bit_rev_fc32(ccorr, fft_size_int);

    const float peak_diag = (ccorr[0] > 1e-6f) ? ccorr[0] : 1.0f;
    for (int lag = -static_cast<int>(CLEAN_KERNEL_RADIUS_SAMPLES);
         lag <= static_cast<int>(CLEAN_KERNEL_RADIUS_SAMPLES); ++lag) {
        const int idx = (lag < 0) ? (fft_size_int + lag) : lag;
        const size_t k = static_cast<size_t>(lag + static_cast<int>(CLEAN_KERNEL_RADIUS_SAMPLES));
        clean_kernel_diag[k] = ccorr[2 * idx] / peak_diag;
    }
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
                // Fast downward tracking when background noise subsides (~1-2 s)
                noise_floor_db += NOISE_FLOOR_ALPHA_DOWN * (rms_db - noise_floor_db);
            } else if (rms_db < trigger_threshold) {
                // Slow upward tracking of ambient noise below the trigger threshold
                noise_floor_db += NOISE_FLOOR_ALPHA_UP * (rms_db - noise_floor_db);
            } else {
                // Active acoustic event / speech: ultra-slow creep so sustained sound does not pull up the threshold and suppress detection
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

        // Multiply by complex conjugate: (re1 + j*im1) * (re2 - j*im2)
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

    // Inverse FFT (IFFT) via complex conjugation prior to forward FFT
    for (size_t i = 0; i < fft_size; ++i) {
        ccorr[2 * i + 1] = -ccorr[2 * i + 1];
    }
    dsps_fft2r_fc32(ccorr, static_cast<int>(fft_size));
    dsps_bit_rev_fc32(ccorr, static_cast<int>(fft_size));
}

void GccPhat::populatePairCorrelations(float pair_corr[AUDIO_NUM_PAIRS][PAIR_CORR_SIZE]) {
    if (pair_corr == nullptr || ccorr == nullptr || fft_size == 0 ||
        band_weight_edge == nullptr || band_weight_diag == nullptr ||
        fft_ch[0] == nullptr || fft_ch[1] == nullptr ||
        fft_ch[2] == nullptr || fft_ch[3] == nullptr) {
        return;
    }
    const int fft_size_int = static_cast<int>(fft_size);

    for (size_t p = 0; p < AUDIO_NUM_PAIRS; ++p) {
        const MicPairDescriptor& pair = MIC_PAIRS[p];
        const float* weights = pair.is_diagonal ? band_weight_diag : band_weight_edge;
        const float weight_sum = pair.is_diagonal ? weight_sum_diag : weight_sum_edge;
        const float norm_factor =
            (weight_sum > 1e-6f) ? weight_sum : static_cast<float>(fft_size);

        computePairCrossCorrelationIfft(pair.mic_i, pair.mic_j, weights);

        for (int lag = -PAIR_CORR_RADIUS; lag <= PAIR_CORR_RADIUS; ++lag) {
            const int idx = (lag < 0) ? (fft_size_int + lag) : lag;
            const size_t k = static_cast<size_t>(lag + PAIR_CORR_RADIUS);
            pair_corr[p][k] = ccorr[2 * idx] / norm_factor;
        }
    }
}

float GccPhat::evaluateSrpPhat(const float pair_corr[AUDIO_NUM_PAIRS][PAIR_CORR_SIZE],
                              float &best_angle_deg, float &best_power) const {
    best_angle_deg = 0.0f;
    best_power = -1.0f;

    if (pair_corr == nullptr || !std::isfinite(c_sound) || c_sound <= 0.0f ||
        !std::isfinite(fs) || fs <= 0.0f || !std::isfinite(d) || d <= 0.0f) {
        return 0.0f;
    }

    const float scale = (d / c_sound) * fs;
    const float deg_to_rad = static_cast<float>(M_PI) / 180.0f;

    for (size_t i = 0; i < SRP_GRID_ANGLES; ++i) {
        const float angle_deg = -180.0f + static_cast<float>(i) * SRP_GRID_STEP_DEG;
        const float theta_rad = angle_deg * deg_to_rad;
        const float u_x = sinf(theta_rad);
        const float u_y = cosf(theta_rad);

        const float t_x = u_x * scale;
        const float t_y = u_y * scale;

        // Theoretical sample lags for the 6 pairs:
        // Pair 0: Right Edge (+Y)            -> +t_y
        // Pair 1: Front Edge (-X)            -> -t_x
        // Pair 2: Left Edge (+Y)             -> +t_y
        // Pair 3: Back Edge (-X)             -> -t_x
        // Pair 4: Diagonal LB -> RF (+X, +Y) -> +(t_x + t_y)
        // Pair 5: Diagonal RB -> LF (-X, +Y) -> +(-t_x + t_y)
        const float lags[AUDIO_NUM_PAIRS] = {
            t_y,
            -t_x,
            t_y,
            -t_x,
            t_x + t_y,
            -t_x + t_y
        };

        float p_sum = 0.0f;
        for (size_t p = 0; p < AUDIO_NUM_PAIRS; ++p) {
            const int round_lag = clamp_corr_lag(static_cast<int>(roundf(lags[p])));
            const size_t k = static_cast<size_t>(round_lag + PAIR_CORR_RADIUS);
            p_sum += pair_corr[p][k];
        }

        const float avg_power = p_sum * (1.0f / static_cast<float>(AUDIO_NUM_PAIRS));
        if (avg_power > best_power) {
            best_power = avg_power;
            best_angle_deg = angle_deg;
        }
    }

    return best_power;
}

DoaResult GccPhat::solveFromSrpSeed(const float pair_corr[AUDIO_NUM_PAIRS][PAIR_CORR_SIZE],
                                    float seed_angle,
                                    float scale,
                                    float rms_db,
                                    float min_confidence,
                                    float out_lags[AUDIO_NUM_PAIRS],
                                    float out_confs[AUDIO_NUM_PAIRS]) {
    DoaResult result = {0.0f, 0.0f, 0.0f, rms_db, false};
    if (out_lags != nullptr) {
        for (size_t p = 0; p < AUDIO_NUM_PAIRS; ++p) out_lags[p] = 0.0f;
    }
    if (out_confs != nullptr) {
        for (size_t p = 0; p < AUDIO_NUM_PAIRS; ++p) out_confs[p] = 0.0f;
    }
    if (pair_corr == nullptr || !std::isfinite(seed_angle) ||
        !std::isfinite(scale) || scale <= 0.0f) {
        return result;
    }

    const float deg_to_rad = static_cast<float>(M_PI) / 180.0f;
    const float theta_rad = seed_angle * deg_to_rad;
    const float u_x = sinf(theta_rad);
    const float u_y = cosf(theta_rad);
    const float t_x = u_x * scale;
    const float t_y = u_y * scale;

    const float theo_lags[AUDIO_NUM_PAIRS] = {
        t_y,
        -t_x,
        t_y,
        -t_x,
        t_x + t_y,
        -t_x + t_y
    };

    float lags[AUDIO_NUM_PAIRS];
    float confs[AUDIO_NUM_PAIRS];

    for (size_t p = 0; p < AUDIO_NUM_PAIRS; ++p) {
        const float theo_lag = theo_lags[p];
        const int center_lag = std::isfinite(theo_lag)
            ? clamp_corr_lag(static_cast<int>(roundf(theo_lag)))
            : 0;
        constexpr int search_radius = 3;

        int best_idx = center_lag;
        float best_val = -INFINITY;

        for (int l = center_lag - search_radius; l <= center_lag + search_radius; ++l) {
            if (l < -PAIR_CORR_RADIUS || l > PAIR_CORR_RADIUS) {
                continue;
            }
            const size_t k = static_cast<size_t>(l + PAIR_CORR_RADIUS);
            const float val = pair_corr[p][k];
            if (val > best_val) {
                best_val = val;
                best_idx = l;
            }
        }

        if (!std::isfinite(best_val)) {
            lags[p] = static_cast<float>(center_lag);
            confs[p] = 0.0f;
            if (out_lags != nullptr) {
                out_lags[p] = lags[p];
            }
            if (out_confs != nullptr) {
                out_confs[p] = confs[p];
            }
            continue;
        }

        // 3-point parabolic interpolation around the local peak
        const int l_m1 = clamp_corr_lag(best_idx - 1);
        const int l_p1 = clamp_corr_lag(best_idx + 1);

        const float alpha = pair_corr[p][static_cast<size_t>(l_m1 + PAIR_CORR_RADIUS)];
        const float beta  = best_val;
        const float gamma = pair_corr[p][static_cast<size_t>(l_p1 + PAIR_CORR_RADIUS)];

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
            const float vertex = beta - 0.25f * (alpha - gamma) * p_offset;
            if (vertex > peak_val) {
                peak_val = vertex;
            }
        }

        lags[p] = static_cast<float>(best_idx) + p_offset;
        confs[p] = (peak_val > 1.0f) ? 1.0f : (peak_val < 0.0f ? 0.0f : peak_val);
        if (out_lags != nullptr) {
            out_lags[p] = lags[p];
        }
        if (out_confs != nullptr) {
            out_confs[p] = confs[p];
        }
    }

    return solveWlsAzimuth(lags, confs, rms_db, min_confidence);
}

DoaResult GccPhat::process(const int32_t* left_channel, const int32_t* right_channel) {
    // 2-channel compatibility wrapper: delegates to quad array processing by mapping
    // stereo left to left-side mics (M2 LF, M3 LB) and right to right-side mics (M0 RB, M1 RF).
    return process(right_channel, right_channel, left_channel, left_channel);
}

DoaResult GccPhat::process(const int32_t* ch0, const int32_t* ch1,
                           const int32_t* ch2, const int32_t* ch3) {
    MultiDoaResult multi_res = processMulti(ch0, ch1, ch2, ch3);
    return multi_res.sources[0];
}

MultiDoaResult GccPhat::processMulti(const int32_t* ch0, const int32_t* ch1,
                                     const int32_t* ch2, const int32_t* ch3) {
    MultiDoaResult multi_res;
    multi_res.count = 0;
    multi_res.rms_db = NOISE_FLOOR_INITIAL_DB;
    multi_res.valid = false;
    for (size_t s = 0; s < DOA_MAX_SOURCES; ++s) {
        multi_res.sources[s] = {0.0f, 0.0f, 0.0f, NOISE_FLOOR_INITIAL_DB, false};
    }

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
        fft_ch[2] == nullptr || fft_ch[3] == nullptr || ccorr == nullptr ||
        !std::isfinite(d) || d <= 0.0f ||
        !std::isfinite(c_sound) || c_sound <= 0.0f ||
        !std::isfinite(fs) || fs <= 0.0f) {
        return multi_res;
    }

    // 1. Remove per-channel DC offset across all 4 channels (using int64_t for exact 24-bit precision)
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

    // 2. 4-channel omnidirectional unwindowed RMS energy and Hann windowing
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
    multi_res.rms_db = (rms > 1e-5f) ? 20.0f * log10f(rms) : NOISE_FLOOR_INITIAL_DB;
    for (size_t s = 0; s < DOA_MAX_SOURCES; ++s) {
        multi_res.sources[s].rms_db = multi_res.rms_db;
    }

    // 3. Adaptive noise floor gating
    if (!updateNoiseFloorAndCheckGate(multi_res.rms_db)) {
        return multi_res;
    }

    // 4. Four forward complex FFTs (X0..X3)
    for (size_t c = 0; c < AUDIO_NUM_CHANNELS; ++c) {
        dsps_fft2r_fc32(fft_ch[c], static_cast<int>(fft_size));
        dsps_bit_rev_fc32(fft_ch[c], static_cast<int>(fft_size));
    }

    // 5. Dual-band cross-correlation and preservation of lag windows on the stack
    float pair_corr[AUDIO_NUM_PAIRS][PAIR_CORR_SIZE];
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

        // Save normalized lag windows [-PAIR_CORR_RADIUS, +PAIR_CORR_RADIUS] on stack for SRP/CLEAN
        for (int lag = -PAIR_CORR_RADIUS; lag <= PAIR_CORR_RADIUS; ++lag) {
            const int idx = (lag < 0) ? (fft_size_int + lag) : lag;
            const size_t k = static_cast<size_t>(lag + PAIR_CORR_RADIUS);
            pair_corr[p][k] = ccorr[2 * idx] / norm_factor;
        }

        const float baseline_len = pair.is_diagonal ? (d * static_cast<float>(M_SQRT2)) : d;
        int max_lag = static_cast<int>(ceilf((baseline_len / c_sound) * fs)) + 1;
        if (max_lag >= fft_size_int) {
            max_lag = fft_size_int - 1;
        }

        int best_idx = 0;
        float best_peak = -INFINITY;
        for (int lag = -max_lag; lag <= max_lag; ++lag) {
            const int idx = (lag < 0) ? (fft_size_int + lag) : lag;
            const float val = ccorr[2 * idx] / norm_factor;
            if (val > best_peak) {
                best_peak = val;
                best_idx = lag;
            }
        }

        // 3-point parabolic interpolation for sub-sample lag and peak amplitude
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

    const float scale = (d / c_sound) * fs;
    const float deg_to_rad = static_cast<float>(M_PI) / 180.0f;
    const float offset_deg = std::isfinite(angle_offset_deg) ? angle_offset_deg : 0.0f;
    const float sec_threshold = (correlation_threshold < DEFAULT_CLEAN_SECONDARY_THRESHOLD)
                                    ? correlation_threshold
                                    : DEFAULT_CLEAN_SECONDARY_THRESHOLD;

    // 6. Pass 0: Estimate primary sound source via 2x2 WLS solver
    DoaResult primary = solveWlsAzimuth(pair_lags_samples, pair_confidences, multi_res.rms_db);
    if (!primary.valid || primary.confidence < correlation_threshold) {
        // Fallback for multi-talker split-brain / chimera lag vectors: seed from global SRP-PHAT peak
        float srp_seed_angle = 0.0f;
        float srp_seed_power = 0.0f;
        evaluateSrpPhat(pair_corr, srp_seed_angle, srp_seed_power);
        if (srp_seed_power >= correlation_threshold && std::isfinite(srp_seed_power)) {
            primary = solveFromSrpSeed(pair_corr, srp_seed_angle, scale, multi_res.rms_db, correlation_threshold);
        }
    }

    multi_res.sources[0] = primary;
    if (!primary.valid || primary.confidence < correlation_threshold) {
        return multi_res;
    }

    multi_res.count = 1;
    multi_res.valid = true;

    // Preserve Pass-0 pairwise diagnostic arrays for the primary source
    float saved_last_pair_lags[AUDIO_NUM_PAIRS];
    float saved_last_pair_confs[AUDIO_NUM_PAIRS];
    float saved_last_pair_psi[AUDIO_NUM_PAIRS];
    float saved_last_pair_weights[AUDIO_NUM_PAIRS];
    std::memcpy(saved_last_pair_lags, last_pair_lags_samples, sizeof(saved_last_pair_lags));
    std::memcpy(saved_last_pair_confs, last_pair_confidences, sizeof(saved_last_pair_confs));
    std::memcpy(saved_last_pair_psi, last_pair_psi, sizeof(saved_last_pair_psi));
    std::memcpy(saved_last_pair_weights, last_pair_weights, sizeof(saved_last_pair_weights));

    // 7. Iterative CLEAN deconvolution for secondary and tertiary sound sources
    const float phi_raw = wrap_angle_180(primary.angle_deg + offset_deg);
    const float theta_raw_rad = phi_raw * deg_to_rad;
    const float u_raw_x = sinf(theta_raw_rad);
    const float u_raw_y = cosf(theta_raw_rad);

    // Subtract primary source acoustic imprint from pair_corr using theoretical WLS planar wavefront
    for (size_t p = 0; p < AUDIO_NUM_PAIRS; ++p) {
        const MicPairDescriptor& pair = MIC_PAIRS[p];
        const float* kernel = pair.is_diagonal ? clean_kernel_diag : clean_kernel_edge;
        const float theo_lag = (pair.dir_x * u_raw_x + pair.dir_y * u_raw_y) * scale;
        const float peak_amp = estimateLocalPeakAmplitude(pair_corr[p], theo_lag);
        subtractCleanKernel(pair_corr, p, kernel, theo_lag, peak_amp);
    }

    // Search for secondary sources in residual correlation field using SRP-PHAT
    for (size_t iter = 1; iter < DOA_MAX_SOURCES; ++iter) {
        float seed_angle = 0.0f;
        float seed_power = 0.0f;
        evaluateSrpPhat(pair_corr, seed_angle, seed_power);

        if (seed_power < sec_threshold || !std::isfinite(seed_power)) {
            break;
        }

        float sec_lags[AUDIO_NUM_PAIRS];
        float sec_confs[AUDIO_NUM_PAIRS];
        DoaResult candidate = solveFromSrpSeed(pair_corr, seed_angle, scale, multi_res.rms_db, sec_threshold, sec_lags, sec_confs);

        // Subtract secondary candidate imprint from pair_corr
        if (candidate.valid) {
            const float phi_cand_raw = wrap_angle_180(candidate.angle_deg + offset_deg);
            const float theta_cand_rad = phi_cand_raw * deg_to_rad;
            const float u_cand_x = sinf(theta_cand_rad);
            const float u_cand_y = cosf(theta_cand_rad);
            for (size_t p = 0; p < AUDIO_NUM_PAIRS; ++p) {
                const MicPairDescriptor& pair = MIC_PAIRS[p];
                const float* kernel = pair.is_diagonal ? clean_kernel_diag : clean_kernel_edge;
                const float theo_lag = (pair.dir_x * u_cand_x + pair.dir_y * u_cand_y) * scale;
                const float peak_amp = estimateLocalPeakAmplitude(pair_corr[p], theo_lag);
                subtractCleanKernel(pair_corr, p, kernel, theo_lag, peak_amp);
            }
        } else {
            for (size_t p = 0; p < AUDIO_NUM_PAIRS; ++p) {
                const MicPairDescriptor& pair = MIC_PAIRS[p];
                const float* kernel = pair.is_diagonal ? clean_kernel_diag : clean_kernel_edge;
                subtractCleanKernel(pair_corr, p, kernel, sec_lags[p], sec_confs[p]);
            }
        }

        if (!candidate.valid || candidate.confidence < sec_threshold) {
            continue;
        }

        // Enforce minimum spatial separation against previously accepted sources
        bool distinct = true;
        for (uint8_t j = 0; j < multi_res.count; ++j) {
            const float diff = fabsf(circular_diff_deg(candidate.angle_deg, multi_res.sources[j].angle_deg));
            if (diff < DOA_MIN_SOURCE_SEPARATION_DEG) {
                distinct = false;
                break;
            }
        }

        if (distinct && multi_res.count < DOA_MAX_SOURCES) {
            multi_res.sources[multi_res.count++] = candidate;
        }
    }

    // Restore Pass-0 pairwise diagnostic arrays so getters reflect the primary source
    std::memcpy(last_pair_lags_samples, saved_last_pair_lags, sizeof(last_pair_lags_samples));
    std::memcpy(last_pair_confidences, saved_last_pair_confs, sizeof(last_pair_confidences));
    std::memcpy(last_pair_psi, saved_last_pair_psi, sizeof(last_pair_psi));
    std::memcpy(last_pair_weights, saved_last_pair_weights, sizeof(last_pair_weights));

    // Sort detected sources descending by confidence
    if (multi_res.count > 1) {
        std::sort(multi_res.sources, multi_res.sources + multi_res.count,
                  [](const DoaResult &a, const DoaResult &b) {
                      const float ca = std::isfinite(a.confidence) ? a.confidence : -1.0f;
                      const float cb = std::isfinite(b.confidence) ? b.confidence : -1.0f;
                      return ca > cb;
                  });
    }

    return multi_res;
}

DoaResult GccPhat::solveWlsAzimuth(const float pair_lags_samples[AUDIO_NUM_PAIRS],
                                   const float pair_confidences[AUDIO_NUM_PAIRS],
                                   float rms_db,
                                   float min_confidence) {
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

    const float eff_threshold = (min_confidence >= 0.0f) ? min_confidence : correlation_threshold;

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

    // 1. Four closed-loop triangle consistency errors (in samples)
    const float dn0 = last_pair_lags_samples[0];
    const float dn1 = last_pair_lags_samples[1];
    const float dn2 = last_pair_lags_samples[2];
    const float dn3 = last_pair_lags_samples[3];
    const float dn4 = last_pair_lags_samples[4];
    const float dn5 = last_pair_lags_samples[5];

    const float tri_err[4] = {
        (pair_valid[0] && pair_valid[1] && pair_valid[5])
            ? fabsf(dn0 + dn1 - dn5)
            : INFINITY, // Triangle 0: (0-1-2) [Pairs 0, 1, 5]
        (pair_valid[3] && pair_valid[2] && pair_valid[5])
            ? fabsf(dn3 + dn2 - dn5)
            : INFINITY, // Triangle 1: (0-3-2) [Pairs 3, 2, 5]
        (pair_valid[0] && pair_valid[3] && pair_valid[4])
            ? fabsf(dn0 - dn3 - dn4)
            : INFINITY, // Triangle 2: (0-1-3) [Pairs 0, 3, 4]
        (pair_valid[4] && pair_valid[1] && pair_valid[2])
            ? fabsf(dn4 + dn1 - dn2)
            : INFINITY  // Triangle 3: (3-1-2) [Pairs 4, 1, 2]
    };

    // 2. Gaussian loop-closure consistency weights psi_p for multipath suppression
    const float sigma_loop = DOA_LOOP_CLOSURE_TOLERANCE_SAMPLES;
    const float inv_two_sigma_sq = 1.0f / (2.0f * sigma_loop * sigma_loop);
    const float min_pair_conf = 0.75f * eff_threshold;

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

    // Reject incoherent reverberation/noise: if no triangle is consistent (e_t <= 2.5)
    // and we lack at least two orthogonal edge pairs above eff_threshold
    const bool any_consistent_triangle =
        (tri_err[0] <= 2.5f) || (tri_err[1] <= 2.5f) ||
        (tri_err[2] <= 2.5f) || (tri_err[3] <= 2.5f);
    const bool has_y_edge = (last_pair_confidences[0] >= eff_threshold) ||
                            (last_pair_confidences[2] >= eff_threshold);
    const bool has_x_edge = (last_pair_confidences[1] >= eff_threshold) ||
                            (last_pair_confidences[3] >= eff_threshold);
    if (!any_consistent_triangle && !(has_y_edge && has_x_edge)) {
        return result;
    }

    // Reject common-mode / diffuse noise: in a 2D planar array, a horizontal wave
    // cannot arrive at all 4 microphones with zero delay simultaneously. Near-zero
    // lags on both orthogonal axes (|dn| < 0.35 samples) indicate vertical incidence,
    // clock/power noise, or diffuse reverberation that would otherwise collapse to a phantom 0.0 deg bearing.
    const bool zero_lag_y = (fabsf(dn0) < 0.35f) && (fabsf(dn2) < 0.35f);
    const bool zero_lag_x = (fabsf(dn1) < 0.35f) && (fabsf(dn3) < 0.35f);
    if (zero_lag_y && zero_lag_x) {
        return result;
    }

    // 3. Assemble and solve 2x2 WLS normal equations: A * u = g
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
    if (mean_conf < eff_threshold) {
        return result;
    }

    const float d2 = d * d;
    const float norm_s_xx = s_xx / d2;
    const float norm_s_yy = s_yy / d2;
    const float norm_s_xy = s_xy / d2;
    const float unreg_det = norm_s_xx * norm_s_yy - norm_s_xy * norm_s_xy;

    if (norm_s_xx < DOA_WLS_MIN_AXIS_OBSERVABILITY ||
        norm_s_yy < DOA_WLS_MIN_AXIS_OBSERVABILITY ||
        unreg_det < DOA_WLS_MIN_UNREG_DET) {
        return result; // Reject numerical rank deficiency (loss of 2D observability along either axis)
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

    constexpr float MIN_U_NORM = 0.25f;
    constexpr float MAX_U_NORM = 1.35f;
    if (!std::isfinite(u_norm) || u_norm < MIN_U_NORM || u_norm > MAX_U_NORM) {
        return result;
    }

    // 4. Full 360-degree azimuth [-180, +180] and circular angle_offset_deg compensation
    const float phi_raw = atan2f(u_x, u_y) * (180.0f / static_cast<float>(M_PI));

    // Physical endfire consistency check: prevent diffuse noise from collapsing
    // into phantom axial bearings (0, +90, -90, 180 deg). A true axial source
    // MUST produce a corresponding physical endfire delay on the parallel edge pairs.
    const float abs_cos_phi = fabsf(cosf(phi_raw * (static_cast<float>(M_PI) / 180.0f)));
    const float abs_sin_phi = fabsf(sinf(phi_raw * (static_cast<float>(M_PI) / 180.0f)));
    const float tau_edge_max = (d / c_sound) * fs;
    constexpr float MIN_ENDFIRE_FRACTION = 0.35f;
    const float min_required_lag = tau_edge_max * MIN_ENDFIRE_FRACTION;

    // If candidate is near the Y-axis (near 0 or 180 deg, abs_cos > 0.90), require
    // at least one Y-axis edge pair (Pair 0 or 2) to exhibit sufficient lag (|dn| >= min_required_lag).
    if (abs_cos_phi > 0.90f) {
        const float max_y_lag = fmaxf(fabsf(dn0), fabsf(dn2));
        if (max_y_lag < min_required_lag) {
            return result; // Reject phantom axial noise collapse (lacks real Y delay)
        }
    }

    // If candidate is near the X-axis (near +90 or -90 deg, abs_sin > 0.90), require
    // at least one X-axis edge pair (Pair 1 or 3) to exhibit sufficient lag (|dn| >= min_required_lag).
    if (abs_sin_phi > 0.90f) {
        const float max_x_lag = fmaxf(fabsf(dn1), fabsf(dn3));
        if (max_x_lag < min_required_lag) {
            return result; // Reject phantom axial noise collapse (lacks real X delay)
        }
    }

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