#include "ui/serial_ui.h"
#include "config/app_config.h"
#include <Wire.h>

void SerialUI::printI2cScan() {
    bool found_device = false;
    Serial.println("I2C scan:");
    for (uint8_t address = 1; address < 127; ++address) {
        Wire.beginTransmission(address);
        if (Wire.endTransmission() == 0) {
            Serial.printf("  Found device at 0x%02X\n", address);
            found_device = true;
        }
    }
    if (!found_device) {
        Serial.println("  No I2C devices found");
    }
}

void SerialUI::printSelfTestReport(const BootSelfTestResult &res, uint32_t sample_rate) {
    Serial.println("\n==================================================");
    Serial.println("           ACOUSTIC LOCATOR BOOT SELF-TEST        ");
    Serial.println("==================================================");
    Serial.printf("[%s] Octal PSRAM : %s (%u KB)\n",
                  res.psram_ok ? "PASS" : "FAIL",
                  res.psram_ok ? "Found" : "Missing",
                  static_cast<unsigned>(res.psram_bytes / 1024));

    if (res.bme_ok) {
        Serial.printf("[PASS] BME280 I2C  : OK (T: %.1fC, P: %.1f hPa)\n",
                      res.bme_temp_c, res.bme_pres_hpa);
    } else {
        Serial.printf("[WARN] BME280 I2C  : NOT FOUND (Using default c=%.1f m/s)\n",
                      DEFAULT_SOUND_SPEED_MPS);
    }

    Serial.printf("[%s] I2S Driver  : %s (%u Hz Dual-Bus Quad)\n",
                  res.i2s_driver_ok ? "PASS" : "FAIL",
                  res.i2s_driver_ok ? "Initialized (I2S0 Master + I2S1 Slave)" : "Failed",
                  static_cast<unsigned>(sample_rate));

    static const char* const mic_labels[AUDIO_NUM_CHANNELS] = {
        "M0 (RB/I2S0L)", "M1 (RF/I2S0R)", "M2 (LF/I2S1L)", "M3 (LB/I2S1R)"
    };
    for (size_t c = 0; c < AUDIO_NUM_CHANNELS; ++c) {
        const bool ok = res.mic_health.ch_ok[c];
        const char* state = ok
            ? "ACTIVE"
            : (res.mic_health.ch_min[c] == res.mic_health.ch_max[c] ? "FLATLINE" : "OUT OF BOUNDS");
        Serial.printf("[%s] %-13s: %s (RMS: %.1f dBFS, min=%ld, max=%ld)\n",
                      ok ? "PASS" : "FAIL", mic_labels[c], state,
                      res.mic_health.ch_rms_db[c],
                      static_cast<long>(res.mic_health.ch_min[c]),
                      static_cast<long>(res.mic_health.ch_max[c]));
    }

    const bool sync_ok = res.i2s_driver_ok &&
                         res.mic_health.bus0_distinct &&
                         res.mic_health.bus1_distinct &&
                         res.mic_health.cross_bus_distinct;
    Serial.printf("[%s] Bus Sync    : Bus0=%s, Bus1=%s, CrossBus=%s (max identical=%u/%u)\n",
                  sync_ok ? "PASS" : "FAIL",
                  res.mic_health.bus0_distinct ? "OK" : "COLLISION",
                  res.mic_health.bus1_distinct ? "OK" : "COLLISION",
                  res.mic_health.cross_bus_distinct ? "OK" : "SHORTED",
                  static_cast<unsigned>(res.mic_health.identical_samples),
                  static_cast<unsigned>(POST_SAMPLE_COUNT));

    Serial.println("==================================================");
    Serial.printf("STATUS: %s\n", res.all_nominal ? "ALL SYSTEMS NOMINAL" : "HARDWARE DEGRADED - CHECK WIRING");
    Serial.println("==================================================\n");
}

void SerialUI::printHardwareDegradedAlert() {
    Serial.println("\n!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    Serial.println("  CRITICAL: BOOT SELF-TEST FAILED - HALTING!");
    Serial.println("  Real-time DOA processing and network streaming");
    Serial.println("  are suspended to prevent corrupt measurements.");
    Serial.println("  Fix hardware wiring and press RST to reboot,");
    Serial.println("  or hold BOOT (GPIO 0) to enter setup mode.");
    Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
}

void SerialUI::printDoaEvent(float angle_deg, float confidence, float calibrated_db) {
    Serial.printf("Zaznan kot: %.1f deg, R: %.2f, dB: %.1f\n",
                  angle_deg, confidence, calibrated_db);
}

void SerialUI::printDspDiagnostics(uint32_t blocks, uint32_t misses, uint32_t max_us,
                                  uint32_t budget_us, uint32_t i2s_fails,
                                  uint32_t disp_drops, uint32_t net_drops,
                                  unsigned int stack_free) {
    Serial.printf(
        "DSP stats: blocks=%lu, deadline_misses=%lu, max_processing=%lu us, "
        "budget=%lu us, i2s_failures=%lu, display_drops=%lu, network_drops=%lu, "
        "stack_free=%u\n",
        static_cast<unsigned long>(blocks),
        static_cast<unsigned long>(misses),
        static_cast<unsigned long>(max_us),
        static_cast<unsigned long>(budget_us),
        static_cast<unsigned long>(i2s_fails),
        static_cast<unsigned long>(disp_drops),
        static_cast<unsigned long>(net_drops),
        stack_free);
}
