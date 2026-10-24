#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <vector>
#include <esp_heap_caps.h>
#include <esp_dsp.h>

// --- Mock ESP-IDF and ESP-DSP for native host testing ---
static int g_mock_fft_calls = 0;
static int g_mock_bit_rev_calls = 0;
static int g_mock_internal_alloc_calls = 0;
static int g_mock_spiram_alloc_calls = 0;
static int g_mock_fail_internal_after = -1; // -1 = never fail internal
static int g_mock_fail_all_after = -1;      // -1 = never fail all
static int g_mock_fft_init_ret = 0;

extern "C" {
    int dsps_fft2r_init_fc32(void*, int) {
        return g_mock_fft_init_ret;
    }

    void dsps_wind_hann_f32(float* w, int n) {
        for (int i = 0; i < n; i++) {
            w[i] = 0.5f * (1.0f - cosf(2.0f * static_cast<float>(M_PI) * i / (n - 1)));
        }
    }

    int dsps_fft2r_fc32(float* data, int n) {
        g_mock_fft_calls++;
        // Standard in-place Cooley-Tukey radix-2 complex FFT (leaves output in bit-reversed order
        // until dsps_bit_rev_fc32 is called, matching ESP-DSP convention).
        for (int len = n; len >= 2; len >>= 1) {
            const int half = len >> 1;
            const float angle_step = -2.0f * static_cast<float>(M_PI) / static_cast<float>(len);
            for (int i = 0; i < n; i += len) {
                for (int j = 0; j < half; ++j) {
                    const float angle = angle_step * static_cast<float>(j);
                    const float wr = cosf(angle);
                    const float wi = sinf(angle);

                    const int u = 2 * (i + j);
                    const int v = 2 * (i + j + half);

                    const float ur = data[u];
                    const float ui = data[u + 1];
                    const float vr = data[v];
                    const float vi = data[v + 1];

                    data[u]     = ur + vr;
                    data[u + 1] = ui + vi;

                    const float dr = ur - vr;
                    const float di = ui - vi;
                    data[v]     = dr * wr - di * wi;
                    data[v + 1] = dr * wi + di * wr;
                }
            }
        }
        return 0;
    }

    int dsps_bit_rev_fc32(float* data, int n) {
        g_mock_bit_rev_calls++;
        int j = 0;
        for (int i = 0; i < n; ++i) {
            if (i < j) {
                const float tr = data[2 * i];
                const float ti = data[2 * i + 1];
                data[2 * i]     = data[2 * j];
                data[2 * i + 1] = data[2 * j + 1];
                data[2 * j]     = tr;
                data[2 * j + 1] = ti;
            }
            int bit = n >> 1;
            while (bit > 0 && (j & bit) != 0) {
                j ^= bit;
                bit >>= 1;
            }
            j |= bit;
        }
        return 0;
    }

    void* heap_caps_malloc(size_t size, uint32_t caps) {
        if ((caps & MALLOC_CAP_INTERNAL) != 0) {
            g_mock_internal_alloc_calls++;
            if (g_mock_fail_all_after >= 0 && g_mock_internal_alloc_calls > g_mock_fail_all_after) {
                return nullptr;
            }
            if (g_mock_fail_internal_after >= 0 &&
                g_mock_internal_alloc_calls > g_mock_fail_internal_after) {
                return nullptr;
            }
        } else if ((caps & MALLOC_CAP_SPIRAM) != 0) {
            g_mock_spiram_alloc_calls++;
            if (g_mock_fail_all_after >= 0) {
                return nullptr;
            }
        }
        return malloc(size);
    }

    void heap_caps_free(void* ptr) {
        free(ptr);
    }
}

#include "audio/gcc_phat.h"
#include "ui/display_filter.h"
#include "audio/post_checker.h"
#include "audio/i2s_sampler.h"
#include "network/rest_protocol.h"

// Test 1: GccPhat adaptive noise floor tracking
void test_gcc_phat_adaptive_noise_floor() {
    printf("[TEST] Running test_gcc_phat_adaptive_noise_floor...\n");

    GccPhat doa(44100.0f, 0.08f, 20.0f);
    assert(doa.init(512));

    // Initially adaptive threshold is disabled by default in GccPhat
    assert(!doa.isAdaptiveThresholdEnabled());
    assert(doa.getSnrMarginDb() == 8.0f);
    assert(!doa.isNoiseFloorInitialized());

    // Enable adaptive thresholding
    doa.setAdaptiveThreshold(true, 10.0f);
    assert(doa.isAdaptiveThresholdEnabled());
    assert(doa.getSnrMarginDb() == 10.0f);

    std::vector<int32_t> left(512, 0);
    std::vector<int32_t> right(512, 0);

    // Initial noise floor starts at -100 dBFS
    assert(doa.getNoiseFloorDb() == -100.0f);

    // Edge case: Feed absolute silence (< -95 dBFS). Should NOT initialize or clamp prematurely
    for (int i = 0; i < 512; i++) {
        left[i] = 0;
        right[i] = 0;
    }
    DoaResult r_silent = doa.process(left.data(), right.data());
    assert(r_silent.rms_db <= -99.0f);
    assert(!doa.isNoiseFloorInitialized());
    assert(doa.getNoiseFloorDb() == -100.0f);

    // Now feed quiet noise (~ -60 dBFS)
    // 2147483648.0 * 10^(-60/20) = 2147483.6
    int32_t quiet_sample = 2147484;
    for (int i = 0; i < 512; i++) {
        left[i] = (i % 2 == 0) ? quiet_sample : -quiet_sample;
        right[i] = left[i];
    }

    DoaResult r = doa.process(left.data(), right.data());
    // Initial valid block should seed noise floor directly to current rms_db
    printf("  Initial RMS: %.2f dBFS, Noise floor: %.2f dBFS, Effective thresh: %.2f dBFS\n",
           r.rms_db, doa.getNoiseFloorDb(), doa.getEffectiveThresholdDb());
    assert(doa.isNoiseFloorInitialized());
    assert(fabsf(doa.getNoiseFloorDb() - r.rms_db) < 0.1f);
    assert(doa.getEffectiveThresholdDb() == doa.getNoiseFloorDb() + 10.0f);
    // Since rms is equal to noise floor, it is below noise_floor + 10dB, so r.valid is false
    assert(!r.valid);

    // Now feed a loud signal (above noise floor + 10dB, e.g. -40 dBFS)
    int32_t loud_sample = 21474836;
    for (int i = 0; i < 512; i++) {
        left[i] = (i % 2 == 0) ? loud_sample : -loud_sample;
        right[i] = left[i];
    }

    float floor_before = doa.getNoiseFloorDb();
    DoaResult r_loud = doa.process(left.data(), right.data());
    printf("  Loud RMS: %.2f dBFS, Noise floor before: %.2f, after: %.2f\n",
           r_loud.rms_db, floor_before, doa.getNoiseFloorDb());

    // Loud signal should be accepted by threshold filter (loud RMS > effective thresh)
    assert(r_loud.rms_db > doa.getEffectiveThresholdDb());
    // Noise floor adaptation during loud signal should be very small (alpha = 0.0005)
    float delta = doa.getNoiseFloorDb() - floor_before;
    assert(delta > 0.0f && delta < 0.05f);

    // Verify updateCalibration sets adaptive params
    doa.updateCalibration(0.08f, 22.0f, 0.40f, -50.0f, false, 6.0f);
    assert(!doa.isAdaptiveThresholdEnabled());
    assert(doa.getEffectiveThresholdDb() == -50.0f);

    doa.updateCalibration(0.08f, 22.0f, 0.40f, -50.0f, true, 7.5f);
    assert(doa.isAdaptiveThresholdEnabled());
    assert(doa.getSnrMarginDb() == 7.5f);

    // Test noise floor tracking slew rates
    // 1. Noise decreases: should track down at alpha = 0.02
    doa.resetNoiseFloor(-50.0f);
    assert(doa.getNoiseFloorDb() == -50.0f);
    assert(doa.isNoiseFloorInitialized());

    // feed -70 dBFS signal
    int32_t lower_sample = 679093; // approx -70 dBFS
    for (int i = 0; i < 512; i++) {
        left[i] = (i % 2 == 0) ? lower_sample : -lower_sample;
        right[i] = left[i];
    }
    doa.process(left.data(), right.data());
    // -50.0 + 0.02 * (-70 - (-50)) = -50.4
    printf("  Noise floor after downward step: %.2f dBFS\n", doa.getNoiseFloorDb());
    assert(doa.getNoiseFloorDb() < -50.2f);

    // 2. Noise creeps up slowly below trigger threshold: should track up at alpha = 0.005
    float pre_up = doa.getNoiseFloorDb();
    // feed signal above pre_up (+4.0 dB) but below trigger threshold (pre_up + 7.5 dB)
    int32_t creep_sample = static_cast<int32_t>(2147483648.0 * powf(10.0f, (pre_up + 4.0f) / 20.0f));
    for (int i = 0; i < 512; i++) {
        left[i] = (i % 2 == 0) ? creep_sample : -creep_sample;
        right[i] = left[i];
    }
    DoaResult r_creep = doa.process(left.data(), right.data());
    printf("  Creep RMS: %.2f dBFS, Noise floor after upward creep step: %.2f dBFS (pre: %.2f)\n",
           r_creep.rms_db, doa.getNoiseFloorDb(), pre_up);
    assert(r_creep.rms_db > pre_up);
    assert(r_creep.rms_db < pre_up + 7.5f);
    assert(doa.getNoiseFloorDb() > pre_up);
    assert(doa.getNoiseFloorDb() - pre_up < 0.05f);

    // 3. Negative snr margin rejection
    doa.setAdaptiveThreshold(true, -5.0f);
    assert(doa.getSnrMarginDb() == 7.5f); // Unchanged

    // 4. Stereo RMS symmetry (Left-only vs Right-only must produce identical RMS)
    for (int i = 0; i < 512; i++) {
        left[i] = (i % 2 == 0) ? loud_sample : -loud_sample;
        right[i] = 0;
    }
    DoaResult r_left_only = doa.process(left.data(), right.data());
    for (int i = 0; i < 512; i++) {
        left[i] = 0;
        right[i] = (i % 2 == 0) ? loud_sample : -loud_sample;
    }
    DoaResult r_right_only = doa.process(left.data(), right.data());
    assert(fabsf(r_left_only.rms_db - r_right_only.rms_db) < 1e-4f);

    // 5. Cold-boot loud event on block 0: noise floor seed must be capped at db_threshold
    doa.resetNoiseFloor(-100.0f);
    assert(!doa.isNoiseFloorInitialized());
    doa.updateCalibration(0.08f, 22.0f, 0.40f, -45.0f, true, 8.0f);
    int32_t cold_loud = static_cast<int32_t>(2147483648.0 * powf(10.0f, -25.0f / 20.0f));
    for (int i = 0; i < 512; i++) {
        left[i] = (i % 2 == 0) ? cold_loud : -cold_loud;
        right[i] = left[i];
    }
    DoaResult r_cold_loud = doa.process(left.data(), right.data());
    assert(doa.isNoiseFloorInitialized());
    assert(fabsf(doa.getNoiseFloorDb() - (-45.0f)) < 1e-4f);
    assert(r_cold_loud.rms_db > doa.getEffectiveThresholdDb());

    // 6. Positive fixed db_threshold safety conversion (e.g. when db_spl_offset = 0.0f)
    doa.updateCalibration(0.08f, 22.0f, 0.40f, 55.0f, false, 8.0f);
    assert(fabsf(doa.getEffectiveThresholdDb() - (55.0f - DEFAULT_DB_SPL_OFFSET)) < 1e-4f);

    printf("  [PASS] test_gcc_phat_adaptive_noise_floor passed.\n");
}

// Test 2: Display needle smoothing filter (EMA with outlier rejection)
void test_display_smoothing_filter() {
    printf("[TEST] Running test_display_smoothing_filter...\n");

    DisplaySmoothingFilter filter;
    assert(!filter.isInitialized());

    // 1. First event initializes directly
    float a1 = filter.update(10.0f, 100);
    assert(filter.isInitialized());
    assert(fabsf(a1 - 10.0f) < 1e-4f);
    assert(fabsf(filter.getSmoothedAngle() - 10.0f) < 1e-4f);

    // 2. Small continuous variation smoothly filtered
    // Raw: 14.0 deg -> diff = 4.0 -> smoothed = 10.0 + 0.25*4.0 = 11.0
    float a2 = filter.update(14.0f, 125);
    assert(fabsf(a2 - 11.0f) < 1e-4f);

    // 3. Isolated outlier jump (>45 deg, e.g. -60.0 deg from +11.0 deg, diff = -71.0)
    // Should be rejected for display needle
    float a3 = filter.update(-60.0f, 150);
    assert(fabsf(a3 - 11.0f) < 1e-4f); // Preserves previous smoothed angle!
    assert(filter.getConsecutiveOutliers() == 1);

    // Next reading returns to normal around 12.0 deg (diff = 1.0)
    float a4 = filter.update(12.0f, 175);
    assert(filter.getConsecutiveOutliers() == 0); // Outlier counter cleared
    assert(fabsf(a4 - (11.0f + 0.25f * 1.0f)) < 1e-4f);

    // 4. Opposing consecutive outliers test:
    // Angle ~11.25. Outlier 1 at +80 deg (reflection right), followed by Outlier 2 at -80 deg (reflection left).
    // They are in opposing directions (+80 vs -80 is 160 deg apart). They should NOT confirm each other!
    float pre_opp = filter.getSmoothedAngle();
    float a_opp1 = filter.update(80.0f, 200);
    assert(fabsf(a_opp1 - pre_opp) < 1e-4f); // Outlier 1 held
    assert(filter.getConsecutiveOutliers() == 1);

    float a_opp2 = filter.update(-80.0f, 225);
    assert(fabsf(a_opp2 - pre_opp) < 1e-4f); // Outlier 2 did NOT confirm Outlier 1! Still held!
    assert(filter.getConsecutiveOutliers() == 1); // Reset to candidate -80

    // Next reading returns to normal around 12.0 deg
    filter.update(12.0f, 250);
    assert(filter.getConsecutiveOutliers() == 0);

    // 5. Confirmed new sound direction (2 consecutive consistent outliers)
    // Current angle ~ 11.4 deg. First reading at -50.0 deg
    float cur = filter.getSmoothedAngle();
    float a5 = filter.update(-50.0f, 300);
    assert(fabsf(a5 - cur) < 1e-4f); // First outlier held
    assert(filter.getConsecutiveOutliers() == 1);

    // Second consecutive reading at -52.0 deg (within 45 deg of -50.0 deg)
    float a6 = filter.update(-52.0f, 325);
    assert(fabsf(a6 - (-52.0f)) < 1e-4f); // Confirmed relocation to new angle!
    assert(filter.getConsecutiveOutliers() == 0);

    // 6. Silence reset (>1500 ms)
    // Event after 2000 ms silence at +45 deg should immediately snap without EMA lag
    float a7 = filter.update(45.0f, 2350);
    assert(fabsf(a7 - 45.0f) < 1e-4f);

    // 7. Full-circle [-180, +180] range & wrapAngle180 checks
    filter.update(120.0f, 2360); // Rear-right quadrant (+120 deg)
    filter.update(120.0f, 2370);
    assert(fabsf(filter.getSmoothedAngle() - 120.0f) < 1e-4f);

    filter.update(-120.0f, 2380); // Rear-left quadrant (-120 deg, 120 deg away on S^1)
    filter.update(-120.0f, 2390);
    assert(fabsf(filter.getSmoothedAngle() - (-120.0f)) < 1e-4f);

    printf("  [PASS] test_display_smoothing_filter passed.\n");
}

// Test 3: POST Microphone Statistical Heuristics
void test_post_checker() {
    printf("[TEST] Running test_post_checker...\n");

    const size_t n = 1024;
    std::vector<int32_t> l(n, 0);
    std::vector<int32_t> r(n, 0);

    // Scenario 1: Both microphones disconnected / flatline at zero
    MicrophoneHealthResult res_flat = evaluate_microphone_health(l.data(), r.data(), n);
    assert(!res_flat.left_ok);
    assert(!res_flat.right_ok);
    assert(!res_flat.stereo_distinct);
    assert(res_flat.left_min == 0 && res_flat.left_max == 0);

    // Scenario 2: Right mic flatlined at stuck constant value (floating line)
    for (size_t i = 0; i < n; ++i) {
        l[i] = static_cast<int32_t>(sinf(i * 0.1f) * 2147483.0f); // ~ -60 dBFS
        r[i] = 42; // flat
    }
    MicrophoneHealthResult res_right_flat = evaluate_microphone_health(l.data(), r.data(), n);
    assert(res_right_flat.left_ok);
    assert(!res_right_flat.right_ok);
    assert(res_right_flat.stereo_distinct);

    // Scenario 3: Channel collision (L/R lines shorted or both tied to same channel)
    for (size_t i = 0; i < n; ++i) {
        l[i] = static_cast<int32_t>(sinf(i * 0.1f) * 2147483.0f);
        r[i] = l[i]; // identical to left!
    }
    MicrophoneHealthResult res_collision = evaluate_microphone_health(l.data(), r.data(), n);
    assert(res_collision.left_ok);
    assert(res_collision.right_ok);
    assert(!res_collision.stereo_distinct); // Collision detected!
    assert(res_collision.identical_samples == n);

    // Scenario 4: Rail short / clipping (RMS > -3 dBFS)
    for (size_t i = 0; i < n; ++i) {
        l[i] = 2147480000; // Almost +1.0 full scale
        r[i] = static_cast<int32_t>(cosf(i * 0.1f) * 2147483.0f);
    }
    MicrophoneHealthResult res_rail = evaluate_microphone_health(l.data(), r.data(), n);
    assert(!res_rail.left_ok); // Rail short rejected!
    assert(res_rail.right_ok);

    // Scenario 5: Nominal independent stereo microphones with ambient room noise
    for (size_t i = 0; i < n; ++i) {
        const int i_int = static_cast<int>(i);
        l[i] = static_cast<int32_t>((sinf(i_int * 0.1f) + ((i_int % 7) - 3) * 0.05f) * 2147483.0f);
        r[i] = static_cast<int32_t>((cosf(i_int * 0.12f) + ((i_int % 11) - 5) * 0.05f) * 2147483.0f);
    }
    MicrophoneHealthResult res_nominal = evaluate_microphone_health(l.data(), r.data(), n);
    assert(res_nominal.left_ok);
    assert(res_nominal.right_ok);
    assert(res_nominal.stereo_distinct);
    assert(res_nominal.left_rms_db > -70.0f && res_nominal.left_rms_db < -50.0f);
    assert(res_nominal.right_rms_db > -70.0f && res_nominal.right_rms_db < -50.0f);

    printf("  [PASS] test_post_checker passed.\n");
}

// Test 3b: Quad-Microphone POST Statistical Heuristics (Bus 0 + Bus 1)
static void test_quad_post_checker() {
    printf("[TEST] Running test_quad_post_checker...\n");

    const size_t n = 1024;
    std::vector<int32_t> ch0(n, 0);
    std::vector<int32_t> ch1(n, 0);
    std::vector<int32_t> ch2(n, 0);
    std::vector<int32_t> ch3(n, 0);

    auto fill_nominal = [&]() {
        for (size_t i = 0; i < n; ++i) {
            const int k = static_cast<int>(i);
            ch0[i] = static_cast<int32_t>((sinf(k * 0.11f) + ((k % 7) - 3) * 0.05f) * 2147483.0f);
            ch1[i] = static_cast<int32_t>((cosf(k * 0.13f) + ((k % 11) - 5) * 0.05f) * 2147483.0f);
            ch2[i] = static_cast<int32_t>((sinf(k * 0.17f + 0.7f) + ((k % 13) - 6) * 0.04f) * 2147483.0f);
            ch3[i] = static_cast<int32_t>((cosf(k * 0.19f + 1.2f) + ((k % 5) - 2) * 0.06f) * 2147483.0f);
        }
    };

    // Scenario 0: Null / empty guards
    MicrophoneHealthResult res_null = evaluate_quad_microphone_health(nullptr, n);
    assert(!res_null.all_ok);
    assert(!res_null.ch_ok[0] && !res_null.ch_ok[1] && !res_null.ch_ok[2] && !res_null.ch_ok[3]);
    assert(!res_null.bus0_distinct && !res_null.bus1_distinct && !res_null.cross_bus_distinct);

    MicrophoneHealthResult res_zero = evaluate_quad_microphone_health(
        ch0.data(), ch1.data(), ch2.data(), ch3.data(), 0);
    assert(!res_zero.all_ok);

    MicrophoneHealthResult res_one_null = evaluate_quad_microphone_health(
        ch0.data(), ch1.data(), nullptr, ch3.data(), n);
    assert(!res_one_null.all_ok);

    // Scenario 1: All 4 channels nominal and independent
    fill_nominal();
    const int32_t* const ptrs[4] = {ch0.data(), ch1.data(), ch2.data(), ch3.data()};
    MicrophoneHealthResult res_nom = evaluate_quad_microphone_health(ptrs, n);
    for (size_t c = 0; c < AUDIO_NUM_CHANNELS; ++c) {
        assert(res_nom.ch_ok[c]);
        assert(res_nom.ch_rms_db[c] > -70.0f && res_nom.ch_rms_db[c] < -50.0f);
        assert(res_nom.ch_min[c] < 0 && res_nom.ch_max[c] > 0);
    }
    assert(res_nom.bus0_distinct);
    assert(res_nom.bus1_distinct);
    assert(res_nom.cross_bus_distinct);
    assert(res_nom.all_ok);
    assert(res_nom.left_ok && res_nom.right_ok && res_nom.stereo_distinct);

    // Scenario 2: Single-channel flatline on Mic 2 (stuck constant value)
    fill_nominal();
    for (size_t i = 0; i < n; ++i) {
        ch2[i] = -128;
    }
    MicrophoneHealthResult res_flat2 = evaluate_quad_microphone_health(
        ch0.data(), ch1.data(), ch2.data(), ch3.data(), n);
    assert(res_flat2.ch_ok[0]);
    assert(res_flat2.ch_ok[1]);
    assert(!res_flat2.ch_ok[2]);
    assert(res_flat2.ch_ok[3]);
    assert(res_flat2.bus0_distinct);
    assert(res_flat2.bus1_distinct);
    assert(res_flat2.cross_bus_distinct);
    assert(!res_flat2.all_ok);

    // Scenario 3: Single-channel rail short on Mic 3 (DC short with ripple and AC clipping)
    fill_nominal();
    for (size_t i = 0; i < n; ++i) {
        ch3[i] = 2147480000 - static_cast<int32_t>((i % 5) * 1000);
    }
    MicrophoneHealthResult res_rail_dc = evaluate_quad_microphone_health(
        ch0.data(), ch1.data(), ch2.data(), ch3.data(), n);
    assert(res_rail_dc.ch_ok[0] && res_rail_dc.ch_ok[1] && res_rail_dc.ch_ok[2]);
    assert(!res_rail_dc.ch_ok[3]);
    assert(res_rail_dc.ch_rms_db[3] >= POST_MIC_RAIL_SHORT_RMS_DB);
    assert(!res_rail_dc.all_ok);

    fill_nominal();
    for (size_t i = 0; i < n; ++i) {
        ch3[i] = (i % 2 == 0) ? 2147480000 : -2147480000;
    }
    MicrophoneHealthResult res_rail_ac = evaluate_quad_microphone_health(
        ch0.data(), ch1.data(), ch2.data(), ch3.data(), n);
    assert(!res_rail_ac.ch_ok[3]);
    assert(res_rail_ac.ch_rms_db[3] >= POST_MIC_RAIL_SHORT_RMS_DB);
    assert(!res_rail_ac.all_ok);

    // Scenario 4: Bus 1 L/R collision (Mic 2 and Mic 3 both jumpered to same L/R phase)
    fill_nominal();
    for (size_t i = 0; i < n; ++i) {
        ch3[i] = ch2[i];
    }
    MicrophoneHealthResult res_bus1_col = evaluate_quad_microphone_health(
        ch0.data(), ch1.data(), ch2.data(), ch3.data(), n);
    assert(res_bus1_col.ch_ok[0] && res_bus1_col.ch_ok[1] &&
           res_bus1_col.ch_ok[2] && res_bus1_col.ch_ok[3]);
    assert(res_bus1_col.bus0_distinct);
    assert(!res_bus1_col.bus1_distinct);
    assert(res_bus1_col.cross_bus_distinct);
    assert(!res_bus1_col.stereo_distinct);
    assert(res_bus1_col.identical_samples == n);
    assert(!res_bus1_col.all_ok);

    // Scenario 5: Bus 0/1 cross-collision (GPIO 6 DIN0 shorted to GPIO 7 DIN1)
    fill_nominal();
    for (size_t i = 0; i < n; ++i) {
        ch2[i] = ch0[i];
        ch3[i] = ch1[i];
    }
    MicrophoneHealthResult res_cross_col = evaluate_quad_microphone_health(
        ch0.data(), ch1.data(), ch2.data(), ch3.data(), n);
    assert(res_cross_col.ch_ok[0] && res_cross_col.ch_ok[1] &&
           res_cross_col.ch_ok[2] && res_cross_col.ch_ok[3]);
    assert(res_cross_col.bus0_distinct);
    assert(res_cross_col.bus1_distinct);
    assert(!res_cross_col.cross_bus_distinct);
    assert(!res_cross_col.stereo_distinct);
    assert(res_cross_col.identical_samples == n);
    assert(!res_cross_col.all_ok);

    // Scenario 6: Collision threshold boundary (< 85% distinct vs >= 85% collision)
    fill_nominal();
    const size_t below_thresh = (n * POST_MIC_MAX_COLLISION_PCT) / 100; // 870 samples (84.96%)
    for (size_t i = 0; i < below_thresh; ++i) {
        ch1[i] = ch0[i];
    }
    MicrophoneHealthResult res_edge_pass = evaluate_quad_microphone_health(
        ch0.data(), ch1.data(), ch2.data(), ch3.data(), n);
    assert(res_edge_pass.bus0_distinct);

    ch1[below_thresh] = ch0[below_thresh]; // 871 samples (85.06% >= 85%)
    MicrophoneHealthResult res_edge_fail = evaluate_quad_microphone_health(
        ch0.data(), ch1.data(), ch2.data(), ch3.data(), n);
    assert(!res_edge_fail.bus0_distinct);
    assert(!res_edge_fail.all_ok);

    // Scenario 7: Dead 24-bit mic with high DC bias (0x5003FF00, ~ -4.08 dBFS) + 1-LSB toggle
    // True AC RMS is -144.5 dBFS (<= -95 dBFS flatline threshold). Must not false-pass from float drift.
    fill_nominal();
    for (size_t i = 0; i < n; ++i) {
        ch3[i] = 0x5003FF00 + static_cast<int32_t>((i & 1) << 8);
    }
    MicrophoneHealthResult res_dc_flat = evaluate_quad_microphone_health(
        ch0.data(), ch1.data(), ch2.data(), ch3.data(), n);
    assert(res_dc_flat.ch_ok[0] && res_dc_flat.ch_ok[1] && res_dc_flat.ch_ok[2]);
    assert(!res_dc_flat.ch_ok[3]);
    assert(res_dc_flat.ch_rms_db[3] <= POST_MIC_FLATLINE_RMS_DB);
    assert(!res_dc_flat.all_ok);

    printf("  [PASS] test_quad_post_checker passed.\n");
}

static void test_gcc_phat_angle_offset() {
    printf("[TEST] Running test_gcc_phat_angle_offset...\n");

    const size_t n = 1024;
    const float fs = 44100.0f;
    const float d = 0.082f;
    GccPhat doa(fs, d, 20.0f);
    assert(doa.init(n));

    // Default angle offset is 0.0 degrees
    assert(std::fabs(doa.getAngleOffsetDeg() - 0.0f) < 1e-4f);

    // Set angle offset within [-180, +180] deg (including > 80 deg for 360-degree array)
    doa.setAngleOffsetDeg(135.0f);
    assert(std::fabs(doa.getAngleOffsetDeg() - 135.0f) < 1e-4f);
    doa.setAngleOffsetDeg(-180.0f);
    assert(std::fabs(doa.getAngleOffsetDeg() - (-180.0f)) < 1e-4f);
    doa.setAngleOffsetDeg(7.0f);
    assert(std::fabs(doa.getAngleOffsetDeg() - 7.0f) < 1e-4f);

    // Out of bounds protection (> 180 or < -180 or NAN should be rejected)
    doa.setAngleOffsetDeg(185.0f);
    assert(std::fabs(doa.getAngleOffsetDeg() - 7.0f) < 1e-4f);
    doa.setAngleOffsetDeg(-185.0f);
    assert(std::fabs(doa.getAngleOffsetDeg() - 7.0f) < 1e-4f);
    doa.setAngleOffsetDeg(NAN);
    assert(std::fabs(doa.getAngleOffsetDeg() - 7.0f) < 1e-4f);

    // Verify updateCalibration updates angle offset
    doa.updateCalibration(d, 20.0f, 0.35f, -60.0f, false, 8.0f, 3.5f);
    assert(std::fabs(doa.getAngleOffsetDeg() - 3.5f) < 1e-4f);

    // Verify mathematical formulation of delay offset compensation
    const float c_sound = doa.getSoundSpeed();
    const float raw_angle = 7.0f;
    const float offset_rad = raw_angle * (static_cast<float>(M_PI) / 180.0f);
    const float delta_t_raw = (d * sinf(offset_rad)) / c_sound; // ~28.8 us

    // When angle_offset is raw_angle (7.0 deg):
    const float delta_t_offset = (d * sinf(offset_rad)) / c_sound;
    const float delta_t_cal = delta_t_raw - delta_t_offset;
    const float ratio = (c_sound * delta_t_cal) / d;
    const float calibrated_angle = asinf(ratio) * (180.0f / static_cast<float>(M_PI));
    assert(std::fabs(calibrated_angle) < 1e-5f); // Exactly 0.0 deg!

    // Verify symmetric full scale reach (-90 and +90):
    // Left edge: raw delay is -d/c + delta_t_offset
    const float left_raw = (-d / c_sound) + delta_t_offset;
    const float left_cal = left_raw - delta_t_offset;
    const float left_ratio = (c_sound * left_cal) / d;
    const float left_angle = asinf(left_ratio) * (180.0f / static_cast<float>(M_PI));
    assert(std::fabs(left_angle - (-90.0f)) < 1e-4f); // Exactly -90.0 deg!

    // Right edge: raw delay is +d/c + delta_t_offset
    const float right_raw = (d / c_sound) + delta_t_offset;
    const float right_cal = right_raw - delta_t_offset;
    const float right_ratio = (c_sound * right_cal) / d;
    const float right_angle = asinf(right_ratio) * (180.0f / static_cast<float>(M_PI));
    assert(std::fabs(right_angle - 90.0f) < 1e-4f); // Exactly +90.0 deg!

    printf("  [PASS] test_gcc_phat_angle_offset passed.\n");
}

static void test_app_config_consistency() {
    printf("[TEST] Running test_app_config_consistency...\n");

    // Verify central defaults and 4-mic hardware topology constants
    assert(AUDIO_NUM_CHANNELS == 4);
    assert(AUDIO_NUM_PAIRS == 6);
    assert(I2S_MASTER_PORT_NUM == 0);
    assert(I2S_SLAVE_PORT_NUM == 1);
    assert(I2S_PORT_NUM == I2S_MASTER_PORT_NUM);
    assert(I2S_BCLK_PIN == 5);
    assert(I2S_WS_PIN == 4);
    assert(I2S_DIN0_PIN == 6);
    assert(I2S_DIN1_PIN == 7);
    assert(I2S_DIN_PIN == I2S_DIN0_PIN);
    assert(std::fabs(DEFAULT_MIC_DISTANCE_M - 0.082f) < 1e-4f);
    assert(DEFAULT_SAMPLE_RATE == 44100);
    assert(DEFAULT_WINDOW_SIZE == 1024);
    assert(AUDIO_MAX_WINDOW_SIZE >= DEFAULT_WINDOW_SIZE);
    assert(DEFAULT_CLEAN_SECONDARY_THRESHOLD > 0.0f && DEFAULT_CLEAN_SECONDARY_THRESHOLD <= 1.0f);
    assert(DOA_WLS_MIN_AXIS_OBSERVABILITY > 0.0f);
    assert(DOA_WLS_MIN_UNREG_DET > 0.0f);
    assert(DISPLAY_IDLE_REFRESH_INTERVAL_MS == 1000);

    // Audio config whitelist validator
    assert(is_supported_audio_config(44100, 512));
    assert(is_supported_audio_config(44100, 1024));
    assert(is_supported_audio_config(44100, 2048));
    assert(is_supported_audio_config(48000, 1024));
    assert(!is_supported_audio_config(96000, 1024));
    assert(!is_supported_audio_config(44100, 256));
    assert(!is_supported_audio_config(44100, 4096));

    // Validation boundaries sanity ([-180, +180] deg for 360-degree array)
    assert(std::fabs(ANGLE_OFFSET_MIN_DEG - (-180.0f)) < 1e-4f);
    assert(std::fabs(ANGLE_OFFSET_MAX_DEG - 180.0f) < 1e-4f);
    assert(SNR_MARGIN_MIN_DB >= 0.0f && SNR_MARGIN_MAX_DB > SNR_MARGIN_MIN_DB);
    assert(SPL_THRESHOLD_MIN_DB < SPL_THRESHOLD_MAX_DB);

    // 360-degree circular radar UI geometry constants
    assert(RADAR_CENTER_X == 64);
    assert(RADAR_CENTER_Y == 44);
    assert(RADAR_RADIUS == 34);

    // Multi-Source constants & data structure verification (Slice 1)
    assert(DOA_MAX_SOURCES == 3);
    assert(std::fabs(DOA_MIN_SOURCE_SEPARATION_DEG - 25.0f) < 1e-4f);
    assert(SRP_GRID_ANGLES == 72);
    assert(std::fabs(SRP_GRID_STEP_DEG - 5.0f) < 1e-4f);
    assert(CLEAN_KERNEL_RADIUS_SAMPLES == 16);
    assert(NETWORK_QUEUE_LENGTH == 128);
    assert(NETWORK_BATCH_MAX_MEASUREMENTS == 24);

    MultiDoaResult multi_test = {};
    assert(multi_test.count == 0);
    assert(!multi_test.valid);
    assert(sizeof(multi_test.sources) == DOA_MAX_SOURCES * sizeof(DoaResult));

    // Verify default-constructed GccPhat inherits central app_config constants
    GccPhat doa_default;
    assert(std::fabs(doa_default.getAngleOffsetDeg() - DEFAULT_ANGLE_OFFSET_DEG) < 1e-4f);
    assert(std::fabs(doa_default.getSnrMarginDb() - DEFAULT_SNR_MARGIN_DB) < 1e-4f);
    assert(std::fabs(doa_default.getSoundSpeed() - DEFAULT_SOUND_SPEED_MPS) < 0.5f);
    assert(std::fabs(GCC_PHAT_MIN_HZ - 300.0f) < 1e-4f);
    assert(std::fabs(GCC_PHAT_EDGE_MAX_HZ - 5000.0f) < 1e-4f);
    assert(std::fabs(GCC_PHAT_DIAG_MAX_HZ - 4500.0f) < 1e-4f);
    assert(std::fabs(GCC_PHAT_RAMP_HZ - 100.0f) < 1e-4f);

    // Verify precomputed CLEAN autocorrelation kernels (Slice 2)
    assert(doa_default.init());
    const float* kernel_edge = doa_default.getCleanKernelEdge();
    const float* kernel_diag = doa_default.getCleanKernelDiag();
    const size_t center = CLEAN_KERNEL_RADIUS_SAMPLES;
    assert(std::fabs(kernel_edge[center] - 1.0f) < 1e-4f);
    assert(std::fabs(kernel_diag[center] - 1.0f) < 1e-4f);
    for (size_t l = 1; l <= CLEAN_KERNEL_RADIUS_SAMPLES; ++l) {
        assert(std::fabs(kernel_edge[center + l] - kernel_edge[center - l]) < 1e-4f);
        assert(std::fabs(kernel_diag[center + l] - kernel_diag[center - l]) < 1e-4f);
    }
    assert(kernel_edge[center + 10] < 0.40f);
    assert(kernel_diag[center + 10] < 0.40f);

    // Verify updateCalibration accepts full [-180, +180] angle offset range
    doa_default.updateCalibration(DEFAULT_MIC_DISTANCE_M, 20.0f, 0.35f, -45.0f, true, 8.0f, 135.0f);
    assert(std::fabs(doa_default.getAngleOffsetDeg() - 135.0f) < 1e-4f);
    doa_default.updateCalibration(DEFAULT_MIC_DISTANCE_M, 20.0f, 0.35f, -45.0f, true, 8.0f, -180.0f);
    assert(std::fabs(doa_default.getAngleOffsetDeg() - (-180.0f)) < 1e-4f);

    printf("  [PASS] test_app_config_consistency passed.\n");
}

static void test_gcc_phat_dual_band_weights() {
    printf("[TEST] Running test_gcc_phat_dual_band_weights...\n");

    // 1. Direct evaluation of computeBandWeight for edge (300-5000 Hz, 100 Hz ramp)
    // and diagonal (300-4500 Hz, 100 Hz ramp) bands
    assert(GccPhat::computeBandWeight(0.0f, GCC_PHAT_MIN_HZ, GCC_PHAT_EDGE_MAX_HZ) == 0.0f);
    assert(GccPhat::computeBandWeight(150.0f, GCC_PHAT_MIN_HZ, GCC_PHAT_EDGE_MAX_HZ) == 0.0f);
    assert(GccPhat::computeBandWeight(200.0f, GCC_PHAT_MIN_HZ, GCC_PHAT_EDGE_MAX_HZ) == 0.0f);
    assert(fabsf(GccPhat::computeBandWeight(250.0f, GCC_PHAT_MIN_HZ, GCC_PHAT_EDGE_MAX_HZ) - 0.5f) < 1e-6f);
    assert(GccPhat::computeBandWeight(300.0f, GCC_PHAT_MIN_HZ, GCC_PHAT_EDGE_MAX_HZ) == 1.0f);
    assert(GccPhat::computeBandWeight(1000.0f, GCC_PHAT_MIN_HZ, GCC_PHAT_EDGE_MAX_HZ) == 1.0f);
    assert(GccPhat::computeBandWeight(2200.0f, GCC_PHAT_MIN_HZ, GCC_PHAT_EDGE_MAX_HZ) == 1.0f);
    assert(GccPhat::computeBandWeight(4500.0f, GCC_PHAT_MIN_HZ, GCC_PHAT_EDGE_MAX_HZ) == 1.0f);
    assert(GccPhat::computeBandWeight(5000.0f, GCC_PHAT_MIN_HZ, GCC_PHAT_EDGE_MAX_HZ) == 1.0f);
    assert(fabsf(GccPhat::computeBandWeight(5050.0f, GCC_PHAT_MIN_HZ, GCC_PHAT_EDGE_MAX_HZ) - 0.5f) < 1e-6f);
    assert(GccPhat::computeBandWeight(5100.0f, GCC_PHAT_MIN_HZ, GCC_PHAT_EDGE_MAX_HZ) == 0.0f);
    assert(GccPhat::computeBandWeight(6000.0f, GCC_PHAT_MIN_HZ, GCC_PHAT_EDGE_MAX_HZ) == 0.0f);

    // Diagonal band (300-4500 Hz, 100 Hz ramp)
    assert(GccPhat::computeBandWeight(200.0f, GCC_PHAT_MIN_HZ, GCC_PHAT_DIAG_MAX_HZ) == 0.0f);
    assert(fabsf(GccPhat::computeBandWeight(250.0f, GCC_PHAT_MIN_HZ, GCC_PHAT_DIAG_MAX_HZ) - 0.5f) < 1e-6f);
    assert(GccPhat::computeBandWeight(300.0f, GCC_PHAT_MIN_HZ, GCC_PHAT_DIAG_MAX_HZ) == 1.0f);
    assert(GccPhat::computeBandWeight(1000.0f, GCC_PHAT_MIN_HZ, GCC_PHAT_DIAG_MAX_HZ) == 1.0f);
    assert(GccPhat::computeBandWeight(4500.0f, GCC_PHAT_MIN_HZ, GCC_PHAT_DIAG_MAX_HZ) == 1.0f);
    assert(fabsf(GccPhat::computeBandWeight(4550.0f, GCC_PHAT_MIN_HZ, GCC_PHAT_DIAG_MAX_HZ) - 0.5f) < 1e-6f);
    assert(GccPhat::computeBandWeight(4600.0f, GCC_PHAT_MIN_HZ, GCC_PHAT_DIAG_MAX_HZ) == 0.0f);
    assert(GccPhat::computeBandWeight(5000.0f, GCC_PHAT_MIN_HZ, GCC_PHAT_DIAG_MAX_HZ) == 0.0f);

    // Invalid parameter guards
    assert(GccPhat::computeBandWeight(NAN, GCC_PHAT_MIN_HZ, GCC_PHAT_EDGE_MAX_HZ) == 0.0f);
    assert(GccPhat::computeBandWeight(1000.0f, 2000.0f, 500.0f) == 0.0f);
    assert(GccPhat::computeBandWeight(1000.0f, 300.0f, 2200.0f, 0.0f) == 0.0f);

    // 2. Verify precomputed tables and normalization sums across supported sample rates & window sizes
    const uint32_t rates[2] = {44100, 48000};
    const size_t windows[3] = {512, 1024, 2048};

    GccPhat doa;
    for (size_t r = 0; r < 2; ++r) {
        for (size_t w = 0; w < 3; ++w) {
            const float fs = static_cast<float>(rates[r]);
            const size_t n = windows[w];
            assert(doa.configure(fs, n));
            assert(doa.getFftSize() == n);

            const float* w_edge = doa.getEdgeBandWeights();
            const float* w_diag = doa.getDiagBandWeights();
            assert(w_edge != nullptr);
            assert(w_diag != nullptr);

            // DC (0 Hz) and Nyquist (fs/2) must be 0.0f
            assert(w_edge[0] == 0.0f && w_diag[0] == 0.0f);
            assert(w_edge[n / 2] == 0.0f && w_diag[n / 2] == 0.0f);

            const float bin_hz = fs / static_cast<float>(n);
            float sum_edge = 0.0f;
            float sum_diag = 0.0f;
            size_t edge_only_bins = 0;

            for (size_t i = 0; i < n; ++i) {
                assert(w_edge[i] >= 0.0f && w_edge[i] <= 1.0f);
                assert(w_diag[i] >= 0.0f && w_diag[i] <= 1.0f);
                assert(w_diag[i] <= w_edge[i] + 1e-6f);

                // Conjugate mirror symmetry across Nyquist
                if (i > 0 && i < n / 2) {
                    assert(fabsf(w_edge[i] - w_edge[n - i]) < 1e-6f);
                    assert(fabsf(w_diag[i] - w_diag[n - i]) < 1e-6f);
                }

                const size_t mirrored_bin = (i <= n / 2) ? i : (n - i);
                const float freq = static_cast<float>(mirrored_bin) * bin_hz;
                if (freq >= 300.0f && freq <= 4500.0f) {
                    assert(fabsf(w_edge[i] - 1.0f) < 1e-6f);
                    assert(fabsf(w_diag[i] - 1.0f) < 1e-6f);
                } else if (freq >= 4600.0f && freq <= 5000.0f) {
                    assert(fabsf(w_edge[i] - 1.0f) < 1e-6f);
                    assert(w_diag[i] == 0.0f);
                    edge_only_bins++;
                } else if (freq <= 200.0f || freq >= 5100.0f) {
                    assert(w_edge[i] == 0.0f);
                    assert(w_diag[i] == 0.0f);
                }

                sum_edge += w_edge[i];
                sum_diag += w_diag[i];
            }

            assert(edge_only_bins > 0);
            assert(fabsf(doa.getEdgeWeightSum() - sum_edge) < 1e-4f);
            assert(fabsf(doa.getDiagWeightSum() - sum_diag) < 1e-4f);
            assert(doa.getEdgeWeightSum() > doa.getDiagWeightSum());
            assert(doa.getDiagWeightSum() > 0.0f);
        }
    }

    printf("  [PASS] test_gcc_phat_dual_band_weights passed.\n");
}

static void test_gcc_phat_quad_rms_and_pipeline() {
    printf("[TEST] Running test_gcc_phat_quad_rms_and_pipeline...\n");

    const size_t n = 1024;
    const float fs = 44100.0f;
    GccPhat doa(fs, DEFAULT_MIC_DISTANCE_M, 20.0f);
    assert(doa.init(n));

    std::vector<int32_t> zeros(n, 0);
    std::vector<int32_t> sig(n, 0);
    const int32_t loud_amp = 21474836; // ~ -40 dBFS
    uint32_t lcg = 0x12345678u;
    for (size_t i = 0; i < n; ++i) {
        lcg = lcg * 1664525u + 1013904223u;
        const float white = (static_cast<float>(static_cast<int32_t>(lcg >> 8)) / 8388608.0f) - 1.0f;
        sig[i] = static_cast<int32_t>(white * static_cast<float>(loud_amp));
    }

    // 1. Null-pointer guards on 4-channel process()
    assert(!doa.process(nullptr, zeros.data(), zeros.data(), zeros.data()).valid);
    assert(!doa.process(zeros.data(), nullptr, zeros.data(), zeros.data()).valid);
    assert(!doa.process(zeros.data(), zeros.data(), nullptr, zeros.data()).valid);
    assert(!doa.process(zeros.data(), zeros.data(), zeros.data(), nullptr).valid);

    // 2. 4-channel omnidirectional RMS symmetry across ch0, ch1, ch2, ch3
    DoaResult r_ch0 = doa.process(sig.data(), zeros.data(), zeros.data(), zeros.data());
    DoaResult r_ch1 = doa.process(zeros.data(), sig.data(), zeros.data(), zeros.data());
    DoaResult r_ch2 = doa.process(zeros.data(), zeros.data(), sig.data(), zeros.data());
    DoaResult r_ch3 = doa.process(zeros.data(), zeros.data(), zeros.data(), sig.data());

    assert(fabsf(r_ch0.rms_db - r_ch1.rms_db) < 1e-5f);
    assert(fabsf(r_ch0.rms_db - r_ch2.rms_db) < 1e-5f);
    assert(fabsf(r_ch0.rms_db - r_ch3.rms_db) < 1e-5f);

    // Full 4-channel excitation must equal 2-channel full stereo excitation (+6.0206 dB over single-ch)
    DoaResult r_all4 = doa.process(sig.data(), sig.data(), sig.data(), sig.data());
    DoaResult r_stereo = doa.process(sig.data(), sig.data());
    assert(fabsf(r_all4.rms_db - r_stereo.rms_db) < 1e-5f);
    assert(fabsf((r_all4.rms_db - r_ch0.rms_db) - 6.0206f) < 1e-3f);

    // 3. Independent DC offset removal across all 4 channels (including large 24-bit DC bias 0x5003FF00)
    std::vector<int32_t> ch0_dc(n), ch1_dc(n), ch2_dc(n), ch3_dc(n);
    for (size_t i = 0; i < n; ++i) {
        ch0_dc[i] = sig[i] + 0x5003FF00;
        ch1_dc[i] = sig[i] - 0x4E123400;
        ch2_dc[i] = sig[i] + 300000000;
        ch3_dc[i] = sig[i] - 250000000;
    }
    DoaResult r_dc = doa.process(ch0_dc.data(), ch1_dc.data(), ch2_dc.data(), ch3_dc.data());
    assert(fabsf(r_dc.rms_db - r_all4.rms_db) < 1e-5f);

    // Verify a silent high-DC-bias line (0x5003FF00 with 1-LSB toggle) does not false-seed adaptive noise floor
    doa.setAdaptiveThreshold(true, 8.0f);
    doa.resetNoiseFloor(-100.0f);
    std::vector<int32_t> dc_flat(n);
    for (size_t i = 0; i < n; ++i) {
        dc_flat[i] = 0x5003FF00 + static_cast<int32_t>((i & 1) << 8);
    }
    DoaResult r_dc_flat = doa.process(dc_flat.data(), dc_flat.data(), dc_flat.data(), dc_flat.data());
    assert(r_dc_flat.rms_db <= -99.0f);
    assert(!doa.isNoiseFloorInitialized());
    doa.setAdaptiveThreshold(false, 8.0f);

    // 4. Verify 4 forward FFTs + 6 inverse FFTs = 10 FFTs when above threshold, and 0 when below
    g_mock_fft_calls = 0;
    g_mock_bit_rev_calls = 0;
    DoaResult r_quiet = doa.process(zeros.data(), zeros.data(), zeros.data(), zeros.data());
    assert(r_quiet.rms_db <= -99.0f);
    assert(g_mock_fft_calls == 0);
    assert(g_mock_bit_rev_calls == 0);

    g_mock_fft_calls = 0;
    g_mock_bit_rev_calls = 0;
    DoaResult r_active = doa.process(sig.data(), sig.data(), sig.data(), sig.data());
    assert(g_mock_fft_calls == static_cast<int>(AUDIO_NUM_CHANNELS + AUDIO_NUM_PAIRS)); // 4 + 6 = 10
    assert(g_mock_bit_rev_calls == static_cast<int>(AUDIO_NUM_CHANNELS + AUDIO_NUM_PAIRS));
    assert(fabsf(r_active.confidence - 1.0f) < 1e-4f);

    // 5. Dual-band spectral selectivity:
    // Synthesize 4 channels with coherent multitone in [1650 Hz, 2150 Hz] (edge-only passband)
    // plus uncorrelated noise across the spectrum so bins outside [1650, 2150 Hz] are incoherent.
    // Diagonal pairs (cutoff 1450 Hz, zero >= 1550 Hz) reject the coherent [1650, 2150 Hz] band completely,
    // whereas edge pairs (cutoff 2200 Hz) capture the coherent [1650, 2150 Hz] band.
    std::vector<int32_t> ch_u[4];
    for (size_t c = 0; c < 4; ++c) {
        ch_u[c].resize(n, 0);
    }
    for (size_t i = 0; i < n; ++i) {
        float coherent_high = 0.0f;
        for (int bin = 38; bin <= 50; ++bin) { // 1636 Hz .. 2153 Hz
            coherent_high += cosf(2.0f * static_cast<float>(M_PI) *
                                  static_cast<float>(bin * static_cast<int>(i)) /
                                  static_cast<float>(n) + static_cast<float>(bin) * 0.37f);
        }
        coherent_high /= 3.0f; // Keep RMS well above -45 dBFS threshold
        for (size_t c = 0; c < 4; ++c) {
            lcg = lcg * 1664525u + 1013904223u;
            const float noise = ((static_cast<float>(static_cast<int32_t>(lcg >> 8)) / 8388608.0f) - 1.0f) * 0.25f;
            ch_u[c][i] = static_cast<int32_t>((coherent_high + noise) * static_cast<float>(loud_amp));
        }
    }
    DoaResult r_edge_only = doa.process(
        ch_u[0].data(), ch_u[1].data(), ch_u[2].data(), ch_u[3].data());
    assert(r_edge_only.rms_db > doa.getEffectiveThresholdDb());
    assert(r_edge_only.confidence > 0.15f && r_edge_only.confidence < 0.45f);
    assert(r_active.confidence > r_edge_only.confidence + 0.5f);

    // 6. Verify configure() SPIRAM fallback and clean rollback on allocation / FFT init failure
    g_mock_internal_alloc_calls = 0;
    g_mock_spiram_alloc_calls = 0;
    g_mock_fail_internal_after = 3; // Allow first 3 buffers in internal RAM, force remaining 5 to SPIRAM
    assert(doa.configure(48000.0f, 2048));
    assert(g_mock_spiram_alloc_calls == 5);
    assert(doa.getFftSize() == 2048);
    g_mock_fail_internal_after = -1;

    // Now force total allocation failure mid-way through configure(44100, 512):
    // Previous valid state (48000 Hz, 2048 samples) must remain intact with zero leaks!
    const float prev_edge_sum = doa.getEdgeWeightSum();
    const float* prev_edge_ptr = doa.getEdgeBandWeights();
    g_mock_internal_alloc_calls = 0;
    g_mock_fail_all_after = 4; // Fail after 4 allocations
    assert(!doa.configure(44100.0f, 512));
    g_mock_fail_all_after = -1;
    assert(doa.getFftSize() == 2048);
    assert(doa.getEdgeBandWeights() == prev_edge_ptr);
    assert(fabsf(doa.getEdgeWeightSum() - prev_edge_sum) < 1e-5f);

    // Force dsps_fft2r_init_fc32 failure: buffers must be freed and previous state preserved
    g_mock_fft_init_ret = -1;
    assert(!doa.configure(44100.0f, 512));
    g_mock_fft_init_ret = 0;
    assert(doa.getFftSize() == 2048);
    assert(doa.getEdgeBandWeights() == prev_edge_ptr);

    printf("  [PASS] test_gcc_phat_quad_rms_and_pipeline passed.\n");
}

static float circular_angle_diff_deg(float a_deg, float b_deg) {
    float diff = fmodf(a_deg - b_deg + 180.0f, 360.0f);
    if (diff < 0.0f) {
        diff += 360.0f;
    }
    return diff - 180.0f;
}

static void synthesize_quad_wavefront(float azimuth_deg, float fs, float d, float c_sound,
                                      size_t n, std::vector<int32_t>& ch0,
                                      std::vector<int32_t>& ch1,
                                      std::vector<int32_t>& ch2,
                                      std::vector<int32_t>& ch3,
                                      float tone_offset_hz = 0.0f,
                                      float phase_offset = 0.0f) {
    ch0.assign(n, 0);
    ch1.assign(n, 0);
    ch2.assign(n, 0);
    ch3.assign(n, 0);

    const float phi_rad = azimuth_deg * (static_cast<float>(M_PI) / 180.0f);
    const float u_x = sinf(phi_rad);
    const float u_y = cosf(phi_rad);

    // Microphone positions:
    // r0 = (+d/2, -d/2), r1 = (+d/2, +d/2), r2 = (-d/2, +d/2), r3 = (-d/2, -d/2)
    const float half_d = 0.5f * d;
    const float r_x[4] = { +half_d, +half_d, -half_d, -half_d };
    const float r_y[4] = { -half_d, +half_d, +half_d, -half_d };

    // Arrival times relative to array center: t_m = -(r_m . u) / c_sound
    float t_arrival[4];
    for (size_t m = 0; m < 4; ++m) {
        t_arrival[m] = -(r_x[m] * u_x + r_y[m] * u_y) / c_sound;
    }

    // Broadband Schroeder-phase multitone spanning 250 Hz .. 4950 Hz (covering wideband bandpass)
    const int num_tones = 128;
    const float f_start = 260.0f;
    const float f_end = 4900.0f;
    const float f_step = (f_end - f_start) / static_cast<float>(num_tones - 1);
    const float amp = 21474836.0f / 8.0f; // Strong signal well above -45 dBFS threshold

    std::vector<int32_t>* ch_ptrs[4] = { &ch0, &ch1, &ch2, &ch3 };

    for (size_t i = 0; i < n; ++i) {
        const float t_base = static_cast<float>(i) / fs;
        for (size_t m = 0; m < 4; ++m) {
            const float t_wave = t_base - t_arrival[m];
            float sample_val = 0.0f;
            for (int k = 0; k < num_tones; ++k) {
                const float freq = f_start + tone_offset_hz + static_cast<float>(k) * f_step;
                const float phase = phase_offset + static_cast<float>(M_PI) *
                                    static_cast<float>(k * (k + 1)) /
                                    static_cast<float>(num_tones);
                sample_val += cosf(2.0f * static_cast<float>(M_PI) * freq * t_wave + phase);
            }
            (*ch_ptrs[m])[i] = static_cast<int32_t>(sample_val * amp);
        }
    }
}

static void synthesize_two_quad_wavefronts(
    float angle1_deg, float amp1,
    float angle2_deg, float amp2,
    float fs, float d, float c_sound, size_t n,
    std::vector<int32_t>& ch0, std::vector<int32_t>& ch1,
    std::vector<int32_t>& ch2, std::vector<int32_t>& ch3) {

    std::vector<int32_t> a0, a1, a2, a3;
    std::vector<int32_t> b0, b1, b2, b3;

    synthesize_quad_wavefront(angle1_deg, fs, d, c_sound, n, a0, a1, a2, a3, 0.0f, 0.0f);
    synthesize_quad_wavefront(angle2_deg, fs, d, c_sound, n, b0, b1, b2, b3, 18.0f, 1.234f);

    ch0.resize(n); ch1.resize(n); ch2.resize(n); ch3.resize(n);
    for (size_t i = 0; i < n; ++i) {
        ch0[i] = static_cast<int32_t>(static_cast<float>(a0[i]) * amp1 + static_cast<float>(b0[i]) * amp2);
        ch1[i] = static_cast<int32_t>(static_cast<float>(a1[i]) * amp1 + static_cast<float>(b1[i]) * amp2);
        ch2[i] = static_cast<int32_t>(static_cast<float>(a2[i]) * amp1 + static_cast<float>(b2[i]) * amp2);
        ch3[i] = static_cast<int32_t>(static_cast<float>(a3[i]) * amp1 + static_cast<float>(b3[i]) * amp2);
    }
}

static void test_gcc_phat_quad_wls_all_quadrants() {
    printf("[TEST] Running test_gcc_phat_quad_wls_all_quadrants...\n");

    const size_t n = 1024;
    const float fs = 44100.0f;
    const float d = DEFAULT_MIC_DISTANCE_M; // 0.082 m
    GccPhat doa(fs, d, 20.0f);
    assert(doa.init(n));
    const float c_sound = doa.getSoundSpeed();

    // Test all 8 required cardinal/intercardinal directions across all 4 quadrants
    // (including both +180 and -180) plus additional oblique angles
    const float test_angles_deg[] = {
        0.0f, 45.0f, 90.0f, 135.0f, 180.0f, -180.0f, -135.0f, -90.0f, -45.0f,
        30.0f, -60.0f, 120.0f, -150.0f
    };
    const size_t num_angles = sizeof(test_angles_deg) / sizeof(test_angles_deg[0]);

    std::vector<int32_t> ch0, ch1, ch2, ch3;

    for (size_t a = 0; a < num_angles; ++a) {
        const float target_deg = test_angles_deg[a];
        synthesize_quad_wavefront(target_deg, fs, d, c_sound, n, ch0, ch1, ch2, ch3);

        DoaResult res = doa.process(ch0.data(), ch1.data(), ch2.data(), ch3.data());
        const float err_deg = fabsf(circular_angle_diff_deg(res.angle_deg, target_deg));

        printf("  Target: %+6.1f deg -> Estimated: %+6.2f deg (err: %.3f deg, conf: %.3f, delay: %+7.2f us)\n",
               target_deg, res.angle_deg, err_deg, res.confidence, res.delay_us);

        assert(res.valid);
        assert(res.confidence > 0.85f);
        assert(err_deg < 1.5f);

        // Verify all 6 pairs have high loop-closure consistency weight on clean wavefronts
        const float* psi = doa.getLastPairLoopWeights();
        for (size_t p = 0; p < AUDIO_NUM_PAIRS; ++p) {
            assert(psi[p] > 0.90f);
        }

        // Verify equivalent X-axis broadside delay telemetry matches angle_deg
        const float expected_delay_us =
            ((d * sinf(res.angle_deg * static_cast<float>(M_PI) / 180.0f)) / c_sound) * 1e6f;
        assert(fabsf(res.delay_us - expected_delay_us) < 1e-3f);
    }

    // Verify circular wrap180 angle_offset_deg compensation across the +-180 deg boundary
    // Case A: Wavefront at -170 deg with +20 deg offset -> (-170 - 20) = -190 -> wraps to +170 deg
    doa.setAngleOffsetDeg(20.0f);
    synthesize_quad_wavefront(-170.0f, fs, d, c_sound, n, ch0, ch1, ch2, ch3);
    DoaResult res_wrap_pos = doa.process(ch0.data(), ch1.data(), ch2.data(), ch3.data());
    assert(res_wrap_pos.valid);
    assert(fabsf(circular_angle_diff_deg(res_wrap_pos.angle_deg, 170.0f)) < 1.5f);
    assert(res_wrap_pos.angle_deg > 168.5f && res_wrap_pos.angle_deg <= 180.0f);

    // Case B: Wavefront at +170 deg with -20 deg offset -> (+170 - (-20)) = +190 -> wraps to -170 deg
    doa.setAngleOffsetDeg(-20.0f);
    synthesize_quad_wavefront(170.0f, fs, d, c_sound, n, ch0, ch1, ch2, ch3);
    DoaResult res_wrap_neg = doa.process(ch0.data(), ch1.data(), ch2.data(), ch3.data());
    assert(res_wrap_neg.valid);
    assert(fabsf(circular_angle_diff_deg(res_wrap_neg.angle_deg, -170.0f)) < 1.5f);
    assert(res_wrap_neg.angle_deg < -168.5f && res_wrap_neg.angle_deg >= -180.0f);

    // Case C: Large rotational offset (-135 deg) via setAngleOffsetDeg() -> wavefront at -110 deg wraps to +25 deg
    doa.setAngleOffsetDeg(-135.0f);
    synthesize_quad_wavefront(-110.0f, fs, d, c_sound, n, ch0, ch1, ch2, ch3);
    DoaResult res_large_off = doa.process(ch0.data(), ch1.data(), ch2.data(), ch3.data());
    assert(res_large_off.valid);
    assert(fabsf(circular_angle_diff_deg(res_large_off.angle_deg, 25.0f)) < 1.5f);

    doa.setAngleOffsetDeg(0.0f);
    printf("  [PASS] test_gcc_phat_quad_wls_all_quadrants passed.\n");
}

static void test_gcc_phat_loop_closure_multipath_rejection() {
    printf("[TEST] Running test_gcc_phat_loop_closure_multipath_rejection...\n");

    const size_t n = 1024;
    const float fs = 44100.0f;
    const float d = DEFAULT_MIC_DISTANCE_M;
    GccPhat doa(fs, d, 20.0f);
    assert(doa.init(n));
    const float c_sound = doa.getSoundSpeed();

    const float test_angles_deg[] = {
        0.0f, 45.0f, 90.0f, 135.0f, 180.0f, -180.0f, -135.0f, -90.0f, -45.0f
    };
    const size_t num_angles = sizeof(test_angles_deg) / sizeof(test_angles_deg[0]);

    std::vector<int32_t> ch0, ch1, ch2, ch3;

    for (size_t a = 0; a < num_angles; ++a) {
        const float target_deg = test_angles_deg[a];
        synthesize_quad_wavefront(target_deg, fs, d, c_sound, n, ch0, ch1, ch2, ch3);

        // 1. Run full FFT/IFFT pipeline to extract baseline sub-sample lags and confidences
        DoaResult clean_res = doa.process(ch0.data(), ch1.data(), ch2.data(), ch3.data());
        assert(clean_res.valid);

        float base_lags[AUDIO_NUM_PAIRS];
        float base_confs[AUDIO_NUM_PAIRS];
        for (size_t p = 0; p < AUDIO_NUM_PAIRS; ++p) {
            base_lags[p] = doa.getLastPairLagsSamples()[p];
            base_confs[p] = doa.getLastPairConfidences()[p];
        }

        // 2. Inject simulated multipath (+8 samples lag corruption) into each pair p in {0..5}
        for (size_t bad_pair = 0; bad_pair < AUDIO_NUM_PAIRS; ++bad_pair) {
            float corrupted_lags[AUDIO_NUM_PAIRS];
            for (size_t p = 0; p < AUDIO_NUM_PAIRS; ++p) {
                corrupted_lags[p] = base_lags[p];
            }
            corrupted_lags[bad_pair] += 8.0f; // Simulated reflection spike (+8 samples)

            DoaResult mp_res = doa.solveWlsAzimuth(corrupted_lags, base_confs, clean_res.rms_db);
            const float err_deg = fabsf(circular_angle_diff_deg(mp_res.angle_deg, target_deg));
            const float* psi = doa.getLastPairLoopWeights();
            const float* wls_w = doa.getLastPairWlsWeights();

            // Verify corrupted pair is heavily suppressed (exp(-8^2 / (2 * 1.5^2)) = 6.6e-7)
            assert(psi[bad_pair] < 1e-4f);
            assert(wls_w[bad_pair] < 1e-4f);

            // Verify all 5 uncorrupted pairs retain high consistency weight via their clean triangle
            for (size_t p = 0; p < AUDIO_NUM_PAIRS; ++p) {
                if (p != bad_pair) {
                    assert(psi[p] > 0.90f);
                }
            }

            // Verify true direct angle is accurately recovered (< 1.5 deg error)
            assert(mp_res.valid);
            assert(err_deg < 1.5f);

            // Also verify non-finite (NAN) lag on bad_pair is completely zero-weighted
            // rather than treated as a fake 0.0f lag
            corrupted_lags[bad_pair] = NAN;
            DoaResult nan_res = doa.solveWlsAzimuth(corrupted_lags, base_confs, clean_res.rms_db);
            assert(nan_res.valid);
            assert(fabsf(circular_angle_diff_deg(nan_res.angle_deg, target_deg)) < 1.5f);
            assert(doa.getLastPairConfidences()[bad_pair] == 0.0f);
            assert(doa.getLastPairLoopWeights()[bad_pair] == 0.0f);
            assert(doa.getLastPairWlsWeights()[bad_pair] == 0.0f);
        }
    }

    // 3. Verify incoherent reverberation / solver edge cases are rejected
    const float bad_lags[AUDIO_NUM_PAIRS] = { 9.0f, -8.0f, -6.0f, 7.0f, 11.0f, -10.0f };
    const float low_confs[AUDIO_NUM_PAIRS] = { 0.30f, 0.30f, 0.30f, 0.30f, 0.30f, 0.30f };
    assert(!doa.solveWlsAzimuth(bad_lags, low_confs, -30.0f).valid);
    assert(!doa.solveWlsAzimuth(nullptr, low_confs, -30.0f).valid);
    assert(!doa.solveWlsAzimuth(bad_lags, nullptr, -30.0f).valid);

    // Verify diffuse zero-lag common-mode noise across all microphones is rejected
    const float zero_lags[AUDIO_NUM_PAIRS] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
    const float high_confs[AUDIO_NUM_PAIRS] = { 0.80f, 0.80f, 0.80f, 0.80f, 0.80f, 0.80f };
    assert(!doa.solveWlsAzimuth(zero_lags, high_confs, -30.0f).valid);

    // Verify mathematical axis collapse anomalies (0.0 deg without true Y transit delay) are rejected
    const float fake_zero_lags[AUDIO_NUM_PAIRS] = { 0.5f, 0.0f, 0.5f, 0.0f, 0.5f, 0.5f };
    assert(!doa.solveWlsAzimuth(fake_zero_lags, high_confs, -30.0f).valid);

    // Verify mathematical axis collapse anomalies (90.0 deg without true X transit delay) are rejected
    const float fake_90_lags[AUDIO_NUM_PAIRS] = { 0.0f, -0.5f, 0.0f, -0.5f, -0.5f, 0.5f };
    assert(!doa.solveWlsAzimuth(fake_90_lags, high_confs, -30.0f).valid);

    // Verify diagnostic arrays are cleared when process() gates out a silent frame
    std::vector<int32_t> silent(n, 0);
    DoaResult silent_res = doa.process(silent.data(), silent.data(), silent.data(), silent.data());
    assert(!silent_res.valid);
    for (size_t p = 0; p < AUDIO_NUM_PAIRS; ++p) {
        assert(doa.getLastPairLagsSamples()[p] == 0.0f);
        assert(doa.getLastPairConfidences()[p] == 0.0f);
        assert(doa.getLastPairLoopWeights()[p] == 0.0f);
        assert(doa.getLastPairWlsWeights()[p] == 0.0f);
    }

    printf("  [PASS] test_gcc_phat_loop_closure_multipath_rejection passed.\n");
}

static void test_display_circular_smoothing_and_outliers() {
    printf("[TEST] Running test_display_circular_smoothing_and_outliers...\n");

    // 1. Direct verification of wrapAngle180 and shortestAngularDiff across [-180, +180]
    assert(fabsf(wrapAngle180(0.0f) - 0.0f) < 1e-6f);
    assert(fabsf(wrapAngle180(180.0f) - 180.0f) < 1e-6f);
    assert(fabsf(wrapAngle180(-180.0f) - (-180.0f)) < 1e-6f);
    assert(fabsf(wrapAngle180(185.0f) - (-175.0f)) < 1e-5f);
    assert(fabsf(wrapAngle180(-185.0f) - 175.0f) < 1e-5f);
    assert(fabsf(wrapAngle180(540.0f) - 180.0f) < 1e-5f);
    assert(fabsf(wrapAngle180(-540.0f) - (-180.0f)) < 1e-5f);
    assert(wrapAngle180(NAN) == 0.0f);
    assert(wrapAngle180(INFINITY) == 0.0f);

    assert(fabsf(shortestAngularDiff(-175.0f, 175.0f) - 10.0f) < 1e-5f);
    assert(fabsf(shortestAngularDiff(175.0f, -175.0f) - (-10.0f)) < 1e-5f);
    assert(fabsf(shortestAngularDiff(-179.0f, 179.0f) - 2.0f) < 1e-5f);
    assert(fabsf(shortestAngularDiff(179.0f, -179.0f) - (-2.0f)) < 1e-5f);
    assert(fabsf(shortestAngularDiff(180.0f, -180.0f) - 0.0f) < 1e-5f);
    assert(fabsf(shortestAngularDiff(-180.0f, 180.0f) - 0.0f) < 1e-5f);
    assert(shortestAngularDiff(NAN, 10.0f) == 0.0f);
    assert(shortestAngularDiff(10.0f, NAN) == 0.0f);

    // 2. Clockwise circular EMA boundary wrapping: +175 deg -> -175 deg (true delta = +10 deg)
    DisplaySmoothingFilter filter;
    uint32_t t_ms = 100;
    float a0 = filter.update(175.0f, t_ms);
    assert(fabsf(a0 - 175.0f) < 1e-5f);
    assert(filter.getConsecutiveOutliers() == 0);

    // Step 1 toward -175 deg: 175 + 0.25 * 10 = +177.5 deg
    t_ms += 25;
    float a1 = filter.update(-175.0f, t_ms);
    assert(filter.getConsecutiveOutliers() == 0);
    assert(fabsf(a1 - 177.5f) < 1e-4f);

    // Step 2 toward -175 deg: 177.5 + 0.25 * 7.5 = +179.375 deg
    t_ms += 25;
    float a2 = filter.update(-175.0f, t_ms);
    assert(filter.getConsecutiveOutliers() == 0);
    assert(fabsf(a2 - 179.375f) < 1e-4f);

    // Step 3 toward -175 deg: 179.375 + 0.25 * 5.625 = 180.78125 -> wraps to -179.21875 deg!
    t_ms += 25;
    float a3 = filter.update(-175.0f, t_ms);
    assert(filter.getConsecutiveOutliers() == 0);
    assert(fabsf(a3 - (-179.21875f)) < 1e-4f);

    // Continue converging to -175 deg; ensure needle always stays in rear hemisphere (|angle| >= 175)
    for (int step = 0; step < 25; ++step) {
        t_ms += 25;
        float ang = filter.update(-175.0f, t_ms);
        assert(filter.getConsecutiveOutliers() == 0);
        assert(fabsf(ang) >= 174.999f && fabsf(ang) <= 180.0f);
    }
    assert(fabsf(filter.getSmoothedAngle() - (-175.0f)) < 0.05f);

    // 3. Counter-clockwise circular EMA boundary wrapping: -175 deg -> +175 deg (true delta = -10 deg)
    filter.reset();
    t_ms = 1000;
    assert(fabsf(filter.update(-175.0f, t_ms) - (-175.0f)) < 1e-5f);

    // Step 1 toward +175 deg: -175 + 0.25 * (-10) = -177.5 deg
    t_ms += 25;
    assert(fabsf(filter.update(175.0f, t_ms) - (-177.5f)) < 1e-4f);
    assert(filter.getConsecutiveOutliers() == 0);

    // Step 2 toward +175 deg: -177.5 + 0.25 * (-7.5) = -179.375 deg
    t_ms += 25;
    assert(fabsf(filter.update(175.0f, t_ms) - (-179.375f)) < 1e-4f);
    assert(filter.getConsecutiveOutliers() == 0);

    // Step 3 toward +175 deg: -179.375 + 0.25 * (-5.625) = -180.78125 -> wraps to +179.21875 deg!
    t_ms += 25;
    assert(fabsf(filter.update(175.0f, t_ms) - 179.21875f) < 1e-4f);
    assert(filter.getConsecutiveOutliers() == 0);

    // 4. Tight boundary transition (+179 deg -> -179 deg, delta = +2 deg)
    filter.reset();
    t_ms = 2000;
    filter.update(179.0f, t_ms);
    t_ms += 25;
    float tight1 = filter.update(-179.0f, t_ms); // 179 + 0.25 * 2 = 179.5
    assert(fabsf(tight1 - 179.5f) < 1e-4f);
    t_ms += 25;
    float tight2 = filter.update(-179.0f, t_ms); // 179.5 + 0.25 * 1.5 = 179.875
    assert(fabsf(tight2 - 179.875f) < 1e-4f);
    t_ms += 25;
    float tight3 = filter.update(-179.0f, t_ms); // 179.875 + 0.25 * 1.125 = 180.15625 -> -179.84375
    assert(fabsf(tight3 - (-179.84375f)) < 1e-4f);

    // 5. Outlier rejection and confirmation across +/-180 deg
    filter.reset();
    t_ms = 3000;
    filter.update(170.0f, t_ms);

    // Moderate jump across +/-180 deg (+170 -> -160 deg, shortest diff = +30 deg <= 45 deg):
    // Must NOT be treated as an outlier; smooths to 170 + 0.25 * 30 = +177.5 deg
    t_ms += 25;
    float mod_cross = filter.update(-160.0f, t_ms);
    assert(filter.getConsecutiveOutliers() == 0);
    assert(fabsf(mod_cross - 177.5f) < 1e-4f);

    // Isolated large outlier jump from +177.5 deg to +90.0 deg (diff = -87.5 deg, |diff| > 45 deg)
    t_ms += 25;
    float out_hold = filter.update(90.0f, t_ms);
    assert(filter.getConsecutiveOutliers() == 1);
    assert(fabsf(out_hold - 177.5f) < 1e-4f); // Held at +177.5 deg

    // Next frame returns near rear (-178.0 deg, diff from +177.5 is +4.5 deg <= 45 deg)
    t_ms += 25;
    float recov = filter.update(-178.0f, t_ms);
    assert(filter.getConsecutiveOutliers() == 0);
    assert(fabsf(recov - 178.625f) < 1e-4f); // 177.5 + 0.25 * 4.5 = +178.625 deg

    // 6. Sustained relocation straddling +/-180 deg:
    // Start at 0 deg (North). Outlier 1 arrives at +170 deg (|diff| = 170 > 45).
    // Outlier 2 arrives at -175 deg (|diff from 0| = 175 > 45, but shortestAngularDiff(-175, +170) = +15 deg <= 45!).
    // Outlier 2 must confirm Outlier 1 across the +/-180 deg seam and snap to -175 deg!
    filter.reset();
    t_ms = 4000;
    assert(fabsf(filter.update(0.0f, t_ms) - 0.0f) < 1e-5f);

    t_ms += 25;
    float reloc1 = filter.update(170.0f, t_ms);
    assert(fabsf(reloc1 - 0.0f) < 1e-5f); // Held on first outlier
    assert(filter.getConsecutiveOutliers() == 1);

    t_ms += 25;
    float reloc2 = filter.update(-175.0f, t_ms);
    assert(fabsf(reloc2 - (-175.0f)) < 1e-5f); // Confirmed relocation across +/-180 seam!
    assert(filter.getConsecutiveOutliers() == 0);

    // 7. Exact antipodal 180 deg reversal straddling +180 and -180:
    filter.reset();
    t_ms = 5000;
    assert(fabsf(filter.update(0.0f, t_ms) - 0.0f) < 1e-5f);
    t_ms += 25;
    assert(fabsf(filter.update(180.0f, t_ms) - 0.0f) < 1e-5f);
    assert(filter.getConsecutiveOutliers() == 1);
    t_ms += 25;
    assert(fabsf(filter.update(-180.0f, t_ms) - (-180.0f)) < 1e-5f);
    assert(filter.getConsecutiveOutliers() == 0);

    // Non-finite input guard
    t_ms += 25;
    assert(fabsf(filter.update(NAN, t_ms) - (-180.0f)) < 1e-5f);

    printf("  [PASS] test_display_circular_smoothing_and_outliers passed.\n");
}

static void test_rest_binary_batch_serialization() {
    printf("[TEST] Running test_rest_binary_batch_serialization...\n");

    assert(MEASUREMENT_BATCH_VERSION == 1);
    assert(MEASUREMENT_BATCH_HEADER_SIZE == 12);
    assert(MEASUREMENT_RECORD_SIZE == 20);

    uint8_t buffer[256];
    std::memset(buffer, 0, sizeof(buffer));

    NetworkMeasurement batch[2];
    batch[0].result = {45.0f, 168.59f, 0.85f, -20.0f, true};
    batch[0].timestamp_ms = 1704067200000ULL;
    batch[0].clock_flags = CLOCK_FLAG_UTC_SYNCHRONIZED;

    batch[1].result = {-90.0f, -238.43f, 0.92f, -15.0f, true};
    batch[1].timestamp_ms = 1704067200023ULL; // +23 ms
    batch[1].clock_flags = CLOCK_FLAG_UTC_SYNCHRONIZED;

    const float db_spl_offset = 96.5f;

    // 1. Nominal batch of 2 records: 12 + 2 * 20 = 52 bytes
    const size_t bytes = serialize_measurement_batch(batch, 2, buffer, sizeof(buffer), db_spl_offset);
    assert(bytes == 52);

    // Verify 12-byte header
    assert(buffer[0] == 1); // Format version
    assert(buffer[1] == 2); // Measurement count
    uint16_t flags_read = static_cast<uint16_t>(buffer[2]) | (static_cast<uint16_t>(buffer[3]) << 8);
    assert(flags_read == CLOCK_FLAG_UTC_SYNCHRONIZED);

    uint64_t base_time_read = 0;
    for (size_t b = 0; b < 8; ++b) {
        base_time_read |= (static_cast<uint64_t>(buffer[4 + b]) << (b * 8));
    }
    assert(base_time_read == 1704067200000ULL);

    // Verify Record 0 (offset 12)
    uint32_t delta0_read = 0;
    for (size_t b = 0; b < 4; ++b) delta0_read |= (static_cast<uint32_t>(buffer[12 + b]) << (b * 8));
    assert(delta0_read == 0);

    float angle0_read = 0.0f;
    uint32_t bits = 0;
    for (size_t b = 0; b < 4; ++b) bits |= (static_cast<uint32_t>(buffer[16 + b]) << (b * 8));
    std::memcpy(&angle0_read, &bits, sizeof(bits));
    assert(fabsf(angle0_read - 45.0f) < 1e-4f);

    float spl0_read = 0.0f;
    bits = 0;
    for (size_t b = 0; b < 4; ++b) bits |= (static_cast<uint32_t>(buffer[24 + b]) << (b * 8));
    std::memcpy(&spl0_read, &bits, sizeof(bits));
    assert(fabsf(spl0_read - (-20.0f + 96.5f)) < 1e-4f);

    float conf0_read = 0.0f;
    bits = 0;
    for (size_t b = 0; b < 4; ++b) bits |= (static_cast<uint32_t>(buffer[28 + b]) << (b * 8));
    std::memcpy(&conf0_read, &bits, sizeof(bits));
    assert(fabsf(conf0_read - 0.85f) < 1e-4f);

    // Verify Record 1 (offset 32)
    uint32_t delta1_read = 0;
    for (size_t b = 0; b < 4; ++b) delta1_read |= (static_cast<uint32_t>(buffer[32 + b]) << (b * 8));
    assert(delta1_read == 23);

    float angle1_read = 0.0f;
    bits = 0;
    for (size_t b = 0; b < 4; ++b) bits |= (static_cast<uint32_t>(buffer[36 + b]) << (b * 8));
    std::memcpy(&angle1_read, &bits, sizeof(bits));
    assert(fabsf(angle1_read - (-90.0f)) < 1e-4f);

    // 2. Error handling: buffer too small
    assert(serialize_measurement_batch(batch, 2, buffer, 51, db_spl_offset) == 0);
    // 3. Error handling: nullptr
    assert(serialize_measurement_batch(nullptr, 2, buffer, sizeof(buffer), db_spl_offset) == 0);
    assert(serialize_measurement_batch(batch, 2, nullptr, sizeof(buffer), db_spl_offset) == 0);
    // 4. Error handling: count = 0 or count > NETWORK_BATCH_MAX_MEASUREMENTS
    assert(serialize_measurement_batch(batch, 0, buffer, sizeof(buffer), db_spl_offset) == 0);
    assert(serialize_measurement_batch(batch, NETWORK_BATCH_MAX_MEASUREMENTS + 1, buffer, sizeof(buffer), db_spl_offset) == 0);
    // 5. Error handling: mismatched clock flags
    batch[1].clock_flags = CLOCK_FLAG_MONOTONIC_UPTIME;
    assert(serialize_measurement_batch(batch, 2, buffer, sizeof(buffer), db_spl_offset) == 0);
    // 6. Error handling: non-monotonic timestamps (clock backwards)
    batch[1].clock_flags = CLOCK_FLAG_UTC_SYNCHRONIZED;
    batch[1].timestamp_ms = 1704067199999ULL; // -1 ms
    assert(serialize_measurement_batch(batch, 2, buffer, sizeof(buffer), db_spl_offset) == 0);

    // 7. Multi-source simultaneous event serialization: 24-record batch (8 frames * 3 simultaneous sources)
    // Frame 0 has 3 simultaneous sources sharing delta_ms = 0.
    std::vector<NetworkMeasurement> multi_batch(NETWORK_BATCH_MAX_MEASUREMENTS);
    const uint64_t multi_base_ms = 1704067200500ULL;
    for (size_t f = 0; f < 8; ++f) {
        const uint64_t frame_time = multi_base_ms + static_cast<uint64_t>(f * 23); // 23 ms audio block
        for (size_t s = 0; s < DOA_MAX_SOURCES; ++s) {
            const size_t idx = f * DOA_MAX_SOURCES + s;
            const float angle = -150.0f + static_cast<float>(idx * 13);
            multi_batch[idx].result = {angle, 100.0f, 0.80f + 0.05f * static_cast<float>(s), -25.0f, true};
            multi_batch[idx].timestamp_ms = frame_time;
            multi_batch[idx].clock_flags = CLOCK_FLAG_UTC_SYNCHRONIZED;
        }
    }

    uint8_t large_buffer[MEASUREMENT_BATCH_HEADER_SIZE + NETWORK_BATCH_MAX_MEASUREMENTS * MEASUREMENT_RECORD_SIZE + 32];
    const size_t multi_bytes = serialize_measurement_batch(
        multi_batch.data(), multi_batch.size(), large_buffer, sizeof(large_buffer), db_spl_offset);
    assert(multi_bytes == MEASUREMENT_BATCH_HEADER_SIZE + NETWORK_BATCH_MAX_MEASUREMENTS * MEASUREMENT_RECORD_SIZE);
    assert(multi_bytes == 492); // 12 + 24 * 20 = 492 bytes

    assert(large_buffer[0] == MEASUREMENT_BATCH_VERSION);
    assert(large_buffer[1] == 24); // count

    // Verify all 3 simultaneous sources in Frame 0 share delta_ms = 0
    for (size_t s = 0; s < DOA_MAX_SOURCES; ++s) {
        const size_t rec_offset = MEASUREMENT_BATCH_HEADER_SIZE + s * MEASUREMENT_RECORD_SIZE;
        uint32_t delta_s = 0;
        for (size_t b = 0; b < 4; ++b) delta_s |= (static_cast<uint32_t>(large_buffer[rec_offset + b]) << (b * 8));
        assert(delta_s == 0); // Exact same timestamp for simultaneous sources

        float angle_s = 0.0f;
        uint32_t angle_bits = 0;
        for (size_t b = 0; b < 4; ++b) angle_bits |= (static_cast<uint32_t>(large_buffer[rec_offset + 4 + b]) << (b * 8));
        std::memcpy(&angle_s, &angle_bits, sizeof(angle_bits));
        assert(fabsf(angle_s - (-150.0f + static_cast<float>(s * 13))) < 1e-4f);
    }

    // Verify Frame 1 sources share delta_ms = 23
    for (size_t s = 0; s < DOA_MAX_SOURCES; ++s) {
        const size_t rec_offset = MEASUREMENT_BATCH_HEADER_SIZE + (DOA_MAX_SOURCES + s) * MEASUREMENT_RECORD_SIZE;
        uint32_t delta_s = 0;
        for (size_t b = 0; b < 4; ++b) delta_s |= (static_cast<uint32_t>(large_buffer[rec_offset + b]) << (b * 8));
        assert(delta_s == 23);
    }

    printf("  [PASS] test_rest_binary_batch_serialization passed.\n");
}

static void test_gcc_phat_srp_phat_coarse_grid() {
    printf("[TEST] Running test_gcc_phat_srp_phat_coarse_grid...\n");

    const size_t n = 1024;
    const float fs = 44100.0f;
    const float d = DEFAULT_MIC_DISTANCE_M; // 0.082 m
    GccPhat doa(fs, d, 20.0f);
    assert(doa.init(n));
    const float c_sound = doa.getSoundSpeed();

    const float test_angles_deg[] = {
        0.0f, 45.0f, 90.0f, 135.0f, 180.0f, -135.0f, -90.0f, -45.0f, 30.0f, -60.0f
    };
    const size_t num_angles = sizeof(test_angles_deg) / sizeof(test_angles_deg[0]);

    std::vector<int32_t> ch0, ch1, ch2, ch3;
    float pair_corr[AUDIO_NUM_PAIRS][PAIR_CORR_SIZE];

    for (size_t a = 0; a < num_angles; ++a) {
        const float target_deg = test_angles_deg[a];
        synthesize_quad_wavefront(target_deg, fs, d, c_sound, n, ch0, ch1, ch2, ch3);

        // Process through GccPhat to compute FFT and populate lag windows
        DoaResult res = doa.process(ch0.data(), ch1.data(), ch2.data(), ch3.data());
        assert(res.valid);

        // Populate pair correlations from the processed frame
        doa.populatePairCorrelations(pair_corr);

        float best_angle = 0.0f;
        float best_power = 0.0f;
        doa.evaluateSrpPhat(pair_corr, best_angle, best_power);

        const float err_deg = fabsf(circular_angle_diff_deg(best_angle, target_deg));
        printf("  Target: %+6.1f deg -> Coarse SRP Angle: %+6.1f deg (err: %.1f deg, power: %.3f)\n",
               target_deg, best_angle, err_deg, best_power);

        // With 5-degree grid resolution, error must be <= 5.0 deg
        assert(err_deg <= 5.0f);
        // Power must be strong on clean synthetic wave (> 0.70)
        assert(best_power > 0.70f);
    }

    printf("  [PASS] test_gcc_phat_srp_phat_coarse_grid passed.\n");
}

static void test_gcc_phat_clean_multisource() {
    printf("[TEST] Running test_gcc_phat_clean_multisource...\n");

    const size_t n = 1024;
    const float fs = 44100.0f;
    const float d = DEFAULT_MIC_DISTANCE_M; // 0.082 m
    GccPhat doa(fs, d, 20.0f);
    assert(doa.init(n));
    const float c_sound = doa.getSoundSpeed();

    std::vector<int32_t> ch0, ch1, ch2, ch3;

    // 1. Single source test: processMulti must extract exactly 1 source matching process()
    synthesize_quad_wavefront(45.0f, fs, d, c_sound, n, ch0, ch1, ch2, ch3);
    MultiDoaResult single_res = doa.processMulti(ch0.data(), ch1.data(), ch2.data(), ch3.data());
    assert(single_res.valid);
    assert(single_res.count == 1);
    assert(fabsf(circular_angle_diff_deg(single_res.sources[0].angle_deg, 45.0f)) < 0.5f);
    assert(single_res.sources[0].confidence > 0.85f);

    DoaResult legacy_res = doa.process(ch0.data(), ch1.data(), ch2.data(), ch3.data());
    assert(legacy_res.valid);
    assert(fabsf(legacy_res.angle_deg - single_res.sources[0].angle_deg) < 1e-4f);
    printf("  Single Source: 45.0 deg -> Extracted count: %d, Angle: %.2f deg (conf: %.3f)\n",
           single_res.count, single_res.sources[0].angle_deg, single_res.sources[0].confidence);

    // 2. Two simultaneous sources: Source 1 at +30.0 deg, Source 2 at -60.0 deg (diff = 90 deg)
    synthesize_two_quad_wavefronts(30.0f, 1.0f, -60.0f, 0.75f, fs, d, c_sound, n, ch0, ch1, ch2, ch3);
    MultiDoaResult two_res = doa.processMulti(ch0.data(), ch1.data(), ch2.data(), ch3.data());
    assert(two_res.valid);
    assert(two_res.count == 2);

    float angle_a = two_res.sources[0].angle_deg;
    float angle_b = two_res.sources[1].angle_deg;
    printf("  Two Sources: [+30.0, -60.0] deg -> Extracted count: %d, Src0: %.2f deg (conf: %.3f), Src1: %.2f deg (conf: %.3f)\n",
           two_res.count, angle_a, two_res.sources[0].confidence, angle_b, two_res.sources[1].confidence);

    // Check that one angle is near +30.0 deg and the other is near -60.0 deg
    const float err_a30 = fabsf(circular_angle_diff_deg(angle_a, 30.0f));
    const float err_a60 = fabsf(circular_angle_diff_deg(angle_a, -60.0f));
    const float err_b30 = fabsf(circular_angle_diff_deg(angle_b, 30.0f));
    const float err_b60 = fabsf(circular_angle_diff_deg(angle_b, -60.0f));

    const bool match_direct = (err_a30 < 7.0f && err_b60 < 7.0f);
    const bool match_reversed = (err_a60 < 7.0f && err_b30 < 7.0f);
    assert(match_direct || match_reversed);
    assert(two_res.sources[0].confidence > 0.40f);
    assert(two_res.sources[1].confidence > 0.25f);

    // Verify minimum spatial separation constraint
    const float separation = fabsf(circular_angle_diff_deg(angle_a, angle_b));
    assert(separation >= DOA_MIN_SOURCE_SEPARATION_DEG);

    printf("  [PASS] test_gcc_phat_clean_multisource passed.\n");
}

static void test_multi_target_display_tracker() {
    printf("[TEST] Running test_multi_target_display_tracker...\n");

    MultiTargetDisplayTracker tracker;
    TrackedTarget targets[DOA_MAX_SOURCES];

    // 1. Initial multi-target detection at t = 1000 ms: two sources at 0.0 deg and 90.0 deg
    float raw1[2] = {0.0f, 90.0f};
    float conf1[2] = {0.95f, 0.85f};
    tracker.update(raw1, conf1, 2, 1000);

    size_t active_count = tracker.getActiveTracks(targets, DOA_MAX_SOURCES);
    assert(active_count == 2);
    assert(fabsf(targets[0].smoothed_angle_deg - 0.0f) < 1e-4f);
    assert(fabsf(targets[1].smoothed_angle_deg - 90.0f) < 1e-4f);

    // 2. Next block at t = 1023 ms with order inverted in raw input: [86.0 deg, 4.0 deg]
    // Nearest-neighbor association must match 4.0 deg -> Track 0 (0.0 deg) and 86.0 deg -> Track 1 (90.0 deg)
    float raw2[2] = {86.0f, 4.0f};
    float conf2[2] = {0.87f, 0.93f};
    tracker.update(raw2, conf2, 2, 1023);

    active_count = tracker.getActiveTracks(targets, DOA_MAX_SOURCES);
    assert(active_count == 2);

    // Track 0 was at 0 deg, updated with 4 deg (EMA alpha=0.25 -> 0 + 0.25 * 4 = 1.0 deg)
    assert(fabsf(targets[0].smoothed_angle_deg - 1.0f) < 0.1f);
    // Track 1 was at 90 deg, updated with 86 deg (EMA alpha=0.25 -> 90 + 0.25 * -4 = 89.0 deg)
    assert(fabsf(targets[1].smoothed_angle_deg - 89.0f) < 0.1f);

    // 3. Test silence timeout: skip 2000 ms (> reset_silence_ms = 1500 ms)
    // New single source at -150.0 deg should reset previous tracks
    float raw3[1] = {-150.0f};
    float conf3[1] = {0.90f};
    tracker.update(raw3, conf3, 1, 3023);

    active_count = tracker.getActiveTracks(targets, DOA_MAX_SOURCES);
    assert(active_count == 1);
    assert(fabsf(targets[0].smoothed_angle_deg - (-150.0f)) < 1e-4f);

    // 4. Test absolute silence timeout without incoming events:
    // After 2000 ms of silence (t = 5023 ms), getActiveTracks(..., now_ms) must prune to 0
    active_count = tracker.getActiveTracks(targets, DOA_MAX_SOURCES, 5023);
    assert(active_count == 0);

    printf("  [PASS] test_multi_target_display_tracker passed.\n");
}

static void test_gcc_phat_rank_deficiency_and_axis_observability() {
    printf("[TEST] Running test_gcc_phat_rank_deficiency_and_axis_observability...\n");

    const float fs = 44100.0f;
    const float d = DEFAULT_MIC_DISTANCE_M; // 0.082 m
    GccPhat doa(fs, d, 20.0f);
    assert(doa.init(1024));
    const float c_sound = doa.getSoundSpeed();
    const float max_lag = (d / c_sound) * fs;

    // 1. Nominal 120 deg sound: both X and Y axes have physical delays
    // theta = 120 deg -> u_x = sin(120) = +0.866, u_y = cos(120) = -0.500
    // theo_lags:
    // Pair 0 (+Y): u_y * max_lag = -0.500 * max_lag
    // Pair 1 (-X): -u_x * max_lag = -0.866 * max_lag
    // Pair 2 (+Y): u_y * max_lag = -0.500 * max_lag
    // Pair 3 (-X): -u_x * max_lag = -0.866 * max_lag
    // Pair 4 (+X, +Y): (u_x + u_y) * max_lag = 0.366 * max_lag
    // Pair 5 (-X, +Y): (-u_x + u_y) * max_lag = -1.366 * max_lag
    float lags_120[AUDIO_NUM_PAIRS] = {
        -0.500f * max_lag,
        -0.866f * max_lag,
        -0.500f * max_lag,
        -0.866f * max_lag,
         0.366f * max_lag,
        -1.366f * max_lag
    };
    float confs_nominal[AUDIO_NUM_PAIRS] = {0.85f, 0.85f, 0.85f, 0.85f, 0.85f, 0.85f};

    DoaResult nom_res = doa.solveWlsAzimuth(lags_120, confs_nominal, -50.0f, 0.45f);
    assert(nom_res.valid);
    assert(fabsf(circular_angle_diff_deg(nom_res.angle_deg, 120.0f)) < 0.5f);

    // 2. Rank deficiency scenario: X-axis pairs (Pair 1, Pair 3) and diagonals drop out
    // Only Y-axis edge pairs (Pair 0, Pair 2) survive with high correlation.
    // Without axis observability check, this collapsed to u_x = 0, u_y < 0 -> EXACTLY 180.0 deg!
    float confs_y_only[AUDIO_NUM_PAIRS] = {0.85f, 0.0f, 0.85f, 0.0f, 0.0f, 0.0f};
    DoaResult y_only_res = doa.solveWlsAzimuth(lags_120, confs_y_only, -50.0f, 0.45f);
    // Must be rejected as invalid due to lack of 2D rank / X-axis observability!
    assert(!y_only_res.valid);

    // 3. Rank deficiency scenario: Y-axis pairs drop out, only X-axis survives.
    // Without axis observability check, this collapsed to u_y = 0 -> -90.0 deg or +90.0 deg!
    float confs_x_only[AUDIO_NUM_PAIRS] = {0.0f, 0.85f, 0.0f, 0.85f, 0.0f, 0.0f};
    DoaResult x_only_res = doa.solveWlsAzimuth(lags_120, confs_x_only, -50.0f, 0.45f);
    assert(!x_only_res.valid);

    // 4. Genuine 180.0 deg sound: wavefront along -Y.
    // Y pairs have max negative lag, X pairs have lag ~ 0 with HIGH confidence!
    float lags_true_180[AUDIO_NUM_PAIRS] = {
        -max_lag, // Pair 0 (+Y)
         0.0f,    // Pair 1 (-X)
        -max_lag, // Pair 2 (+Y)
         0.0f,    // Pair 3 (-X)
        -max_lag, // Pair 4
        -max_lag  // Pair 5
    };
    float confs_true_180[AUDIO_NUM_PAIRS] = {0.90f, 0.90f, 0.90f, 0.90f, 0.90f, 0.90f};
    DoaResult true_180_res = doa.solveWlsAzimuth(lags_true_180, confs_true_180, -50.0f, 0.45f);
    assert(true_180_res.valid);
    assert(fabsf(fabsf(true_180_res.angle_deg) - 180.0f) < 0.5f);

    printf("  [PASS] test_gcc_phat_rank_deficiency_and_axis_observability passed.\n");
}

int main() {
    test_gcc_phat_adaptive_noise_floor();
    test_display_smoothing_filter();
    test_post_checker();
    test_quad_post_checker();
    test_gcc_phat_angle_offset();
    test_app_config_consistency();
    test_gcc_phat_dual_band_weights();
    test_gcc_phat_quad_rms_and_pipeline();
    test_gcc_phat_quad_wls_all_quadrants();
    test_gcc_phat_loop_closure_multipath_rejection();
    test_display_circular_smoothing_and_outliers();
    test_rest_binary_batch_serialization();
    test_gcc_phat_srp_phat_coarse_grid();
    test_gcc_phat_clean_multisource();
    test_multi_target_display_tracker();
    test_gcc_phat_rank_deficiency_and_axis_observability();
    printf("\nAll unit tests passed successfully!\n");
    return 0;
}
