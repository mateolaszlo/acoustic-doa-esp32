#pragma once
#include <esp_dsp.h>
#include <cmath>
#include <cstddef>
#include <cstring>
#include "config/app_config.h"

#define N_SAMPLES AUDIO_DEFAULT_WINDOW_SIZE // Število vzorcev (potenca 2 za FFT)

struct DoaResult {
    float angle_deg;       // Azimut v stopinjah [-180, +180] (4-mic) oz. [-90, +90] (2-mic)
    float delay_us;        // Zakasnitev v mikrosekundah
    float confidence;      // Normaliziran korelacijski vrh [0, 1]
    float rms_db;          // Glasnost v dBFS
    bool valid;
};

struct MultiDoaResult {
    DoaResult sources[DOA_MAX_SOURCES]; // Detektirani zvočni viri, razvrščeni padajoče po zaupanju
    uint8_t count;                      // Število veljavnih virov (0..DOA_MAX_SOURCES)
    float rms_db;                       // Vsesmerna energija bloka v dBFS
    bool valid;                         // Resnično, če je vsaj en vir veljaven
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

    // Initializes the fixed 1024-sample FFT window and working buffers in internal SRAM.
    bool init();
    // Initializes the FFT and all working buffers (window_size must be AUDIO_FFT_WINDOW_SIZE).
    bool init(size_t window_size);
    // In-place sample rate reconfiguration (44.1k <-> 48k) without heap churn.
    bool configure(float sample_rate);
    // Backward-compatible configure taking window_size (must be AUDIO_FFT_WINDOW_SIZE).
    bool configure(float sample_rate, size_t window_size);
    DoaResult process(const int32_t* left_channel, const int32_t* right_channel);
    DoaResult process(const int32_t* ch0, const int32_t* ch1,
                      const int32_t* ch2, const int32_t* ch3);
    /**
     * @brief Process synchronized 4-channel audio block for multiple concurrent sound sources.
     * Combines pairwise GCC-PHAT, Pass-0 chimera fallback via SRP-PHAT seeding, and iterative
     * CLEAN deconvolution with WLS planar wavefront subtraction to extract up to DOA_MAX_SOURCES sources.
     */
    MultiDoaResult processMulti(const int32_t* ch0, const int32_t* ch1,
                                const int32_t* ch2, const int32_t* ch3);

    /**
     * @brief Compute Steered Response Power (SRP-PHAT) over a 360-degree coarse angular grid (72 directions).
     * @param pair_corr Preserved correlation lag matrices [-PAIR_CORR_RADIUS, +PAIR_CORR_RADIUS] for all 6 pairs.
     * @param[out] best_angle_deg Output azimuth in degrees corresponding to maximum steered power.
     * @param[out] best_power Output maximum average steered response power in [0, 1].
     * @return Best steered response power.
     */
    float evaluateSrpPhat(const float pair_corr[AUDIO_NUM_PAIRS][PAIR_CORR_SIZE],
                          float &best_angle_deg, float &best_power) const;

    /**
     * @brief Populate normalized cross-correlation lag windows for all 6 microphone pairs.
     */
    void populatePairCorrelations(float pair_corr[AUDIO_NUM_PAIRS][PAIR_CORR_SIZE]);

    /**
     * @brief Solve 2D horizontal azimuth via 2x2 Weighted Least-Squares (WLS) from 6-pair delays.
     * Evaluates 4-triangle closed loop errors, suppresses multipath outliers, and enforces
     * 2D rank observability guards against numerical axis collapse.
     */
    DoaResult solveWlsAzimuth(const float pair_lags_samples[AUDIO_NUM_PAIRS],
                              const float pair_confidences[AUDIO_NUM_PAIRS],
                              float rms_db,
                              float min_confidence = -1.0f);
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
    static float computeBandWeight(float frequency_hz, float min_hz,
                                   float max_hz, float ramp_hz = GCC_PHAT_RAMP_HZ);
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

    size_t getFftSize() const { return fft_size; }
    const float* getEdgeBandWeights() const { return band_weight_edge; }
    const float* getDiagBandWeights() const { return band_weight_diag; }
    float getEdgeWeightSum() const { return weight_sum_edge; }
    float getDiagWeightSum() const { return weight_sum_diag; }
    const float* getCleanKernelEdge() const { return clean_kernel_edge; }
    const float* getCleanKernelDiag() const { return clean_kernel_diag; }
    const float* getLastPairLagsSamples() const { return last_pair_lags_samples; }
    const float* getLastPairConfidences() const { return last_pair_confidences; }
    const float* getLastPairLoopWeights() const { return last_pair_psi; }
    const float* getLastPairWlsWeights() const { return last_pair_weights; }

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
    float* band_weight_edge;
    float* band_weight_diag;
    float* fft_ch[4];
    float* ccorr;
    float weight_sum_edge;
    float weight_sum_diag;
    float last_pair_lags_samples[AUDIO_NUM_PAIRS];
    float last_pair_confidences[AUDIO_NUM_PAIRS];
    float last_pair_psi[AUDIO_NUM_PAIRS];
    float last_pair_weights[AUDIO_NUM_PAIRS];

    float clean_kernel_edge[2 * CLEAN_KERNEL_RADIUS_SAMPLES + 1];
    float clean_kernel_diag[2 * CLEAN_KERNEL_RADIUS_SAMPLES + 1];

    bool updateNoiseFloorAndCheckGate(float rms_db);
    void computePairCrossCorrelationIfft(size_t mic_i, size_t mic_j, const float* weights);
    void precomputeCleanKernels();
    /**
     * @brief Extract candidate lags guided by SRP angular ray and solve azimuth via WLS.
     * Evaluates local 3-point parabolic peak within +/- 3 samples of theoretical plane wave delays.
     */
    DoaResult solveFromSrpSeed(const float pair_corr[AUDIO_NUM_PAIRS][PAIR_CORR_SIZE],
                               float seed_angle,
                               float scale,
                               float rms_db,
                               float min_confidence,
                               float out_lags[AUDIO_NUM_PAIRS] = nullptr,
                               float out_confs[AUDIO_NUM_PAIRS] = nullptr);
    void releaseBuffers();
};