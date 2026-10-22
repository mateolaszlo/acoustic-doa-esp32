#pragma once
#include <Arduino.h>
#include <cstdint>
#include "ui/radar_ui.h"

class SerialUI {
public:
    static void printSelfTestReport(const BootSelfTestResult &res, uint32_t sample_rate);
    static void printHardwareDegradedAlert();
    static void printDoaEvent(float angle_deg, float confidence, float calibrated_db);
    static void printI2cScan();
    static void printDspDiagnostics(uint32_t blocks, uint32_t misses, uint32_t max_us,
                                    uint32_t budget_us, uint32_t i2s_fails,
                                    uint32_t disp_drops, uint32_t net_drops,
                                    unsigned int stack_free);
};
