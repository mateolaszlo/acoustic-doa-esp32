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

    // Initializes the default window (or the size supplied to the
    // size-aware constructor).
    bool init();
    // Initializes the FFT and all working buffers for a power-of-two window.
    bool init(size_t window_size);
    // Atomically switches the sample rate and FFT size after successful setup.
    bool configure(float sample_rate, size_t window_size);
    DoaResult process(const int32_t* left_channel, const int32_t* right_channel);
    DoaResult process(const int32_t* ch0, const int32_t* ch1,
                      const int32_t* ch2, const int32_t* ch3);
    MultiDoaResult processMulti(const int32_t* ch0, const int32_t* ch1,
                                const int32_t* ch2, const int32_t* ch3);
    float evaluateSrpPhat(const float pair_corr[AUDIO_NUM_PAIRS][PAIR_CORR_SIZE],
                          float &best_angle_deg, float &best_power) const;
    void populatePairCorrelations(float pair_corr[AUDIO_NUM_PAIRS][PAIR_CORR_SIZE]);
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
    void releaseBuffers();
};