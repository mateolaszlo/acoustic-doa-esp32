#pragma once
#include <cstdint>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <SPI.h>
#include "config/app_config.h"
#include "audio/post_checker.h"

struct BootSelfTestResult {
    bool psram_ok = false;
    size_t psram_bytes = 0;
    bool bme_ok = false;
    float bme_temp_c = 0.0f;
    float bme_pres_hpa = 0.0f;
    bool i2s_driver_ok = false;
    MicrophoneHealthResult mic_health{};
    bool all_nominal = false;
};

class RadarDisplay {
public:
    RadarDisplay();

    bool init();
    void drawRadar(float angle_deg, float temp_c, float hum, float pres, float c_sound,
                   float db_spl, bool api_available, bool adaptive_active = false,
                   float snr_db = 0.0f, float snr_margin_db = DEFAULT_SNR_MARGIN_DB,
                   uint32_t sample_rate = DEFAULT_SAMPLE_RATE);
    /**
     * @brief Render multi-target 360-degree radar scope and telemetry dashboard on ST7735 TFT.
     * @param angles_deg Array of target azimuths in degrees [-180, +180].
     * @param confidences Array of target confidence values in [0, 1].
     * @param count Number of active targets to render (0..DOA_MAX_SOURCES).
     * @param slot_ids Optional array of persistent track slot IDs (0=SPL colored, 1=Cyan, 2=Magenta).
     *                 If nullptr, falls back to array index order.
     * @param sample_rate Audio sampling rate in Hz (e.g. 44100 or 48000).
     */
    void drawRadarMulti(const float* angles_deg, const float* confidences, size_t count,
                        float temp_c, float hum, float pres, float c_sound,
                        float db_spl, bool api_available, bool adaptive_active = false,
                        float snr_db = 0.0f, float snr_margin_db = DEFAULT_SNR_MARGIN_DB,
                        const uint8_t* slot_ids = nullptr,
                        uint32_t sample_rate = DEFAULT_SAMPLE_RATE);
    void drawBootPrompt(uint32_t remaining_ms, uint32_t total_ms, const BootSelfTestResult &post);
    void drawHardwareDegraded(const BootSelfTestResult &post);
    void drawSetupPortal();
    bool isInitialized() const { return initialized; }
    bool hasActiveDecayBlips(uint32_t now_ms) const;

private:
    static constexpr size_t MAX_DECAY_BLIPS = 16;
    static constexpr uint32_t PHOSPHOR_DECAY_DURATION_MS = 500;
    struct RadarHitBlip {
        float angle_deg;
        uint32_t timestamp_ms;
        bool active;
    };

    SPIClass tft_spi;
    Adafruit_ST7735 display;
    GFXcanvas16 canvas;
    bool initialized;
    RadarHitBlip hit_blips[MAX_DECAY_BLIPS];
    size_t hit_blip_head;
};
