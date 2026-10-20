#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <vector>

// --- Mock ESP-IDF and ESP-DSP for native host testing ---
extern "C" {
    int dsps_fft2r_init_fc32(void*, int) { return 0; }
    void dsps_wind_hann_f32(float* w, int n) {
        for (int i = 0; i < n; i++) w[i] = 0.5f * (1.0f - cosf(2.0f * M_PI * i / (n - 1)));
    }
    int dsps_fft2r_fc32(float*, int) { return 0; }
    int dsps_bit_rev_fc32(float*, int) { return 0; }
    void* heap_caps_malloc(size_t size, uint32_t) { return malloc(size); }
    void heap_caps_free(void* ptr) { free(ptr); }
}

#include "../src/gcc_phat.h"
#include "../src/display_filter.h"

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
    // feed signal above pre_up but below trigger threshold (pre_up + 7.5 dB)
    int32_t creep_sample = static_cast<int32_t>(2147483648.0 * powf(10.0f, (pre_up + 8.0f) / 20.0f));
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

    // 7. Clamp checks
    filter.update(120.0f, 2360); // Above 90
    filter.update(120.0f, 2370);
    assert(filter.getSmoothedAngle() == 90.0f);

    filter.update(-120.0f, 2380); // Below -90
    filter.update(-120.0f, 2390);
    assert(filter.getSmoothedAngle() == -90.0f);

    printf("  [PASS] test_display_smoothing_filter passed.\n");
}

int main() {
    test_gcc_phat_adaptive_noise_floor();
    test_display_smoothing_filter();
    printf("\nAll unit tests passed successfully!\n");
    return 0;
}
