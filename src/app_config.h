#pragma once
#include <cstdint>
#include <cstddef>

// ==============================================================================
// 1. HARDWARE PINOUT & BUS SPEEDS
// ==============================================================================
// I2S Digital Microphones (ESP32-S3 Dual-Bus Quad Array)
constexpr int I2S_MASTER_PORT_NUM       = 0;
constexpr int I2S_SLAVE_PORT_NUM        = 1;
constexpr int I2S_PORT_NUM              = I2S_MASTER_PORT_NUM;
constexpr int I2S_BCLK_PIN              = 5;
constexpr int I2S_WS_PIN                = 4;
constexpr int I2S_DIN0_PIN              = 6;
constexpr int I2S_DIN1_PIN              = 7;
constexpr int I2S_DIN_PIN               = I2S_DIN0_PIN;

// I2C Environmental Sensor (BME280)
constexpr int I2C_SDA_PIN               = 8;
constexpr int I2C_SCL_PIN               = 9;
constexpr uint32_t I2C_BUS_SPEED_HZ     = 400000; // 400 kHz Fast I2C
constexpr uint8_t BME280_ADDR_PRIMARY   = 0x76;
constexpr uint8_t BME280_ADDR_SECONDARY = 0x77;

// SPI TFT Display (ST7735 / KMR-1.8 128x160)
constexpr int TFT_CS_PIN                = 10;
constexpr int TFT_MOSI_PIN              = 11;
constexpr int TFT_SCLK_PIN              = 12;
constexpr int TFT_DC_PIN                = 13;
constexpr int TFT_RST_PIN               = 14;
constexpr bool TFT_INVERT_COLORS        = false;
constexpr int TFT_WIDTH                 = 128;
constexpr int TFT_HEIGHT                = 160;

// Setup Button (Active-low BOOT button on GPIO 0)
constexpr int SETUP_BUTTON_PIN          = 0;

// ==============================================================================
// 2. AUDIO & BUFFER SIZING
// ==============================================================================
constexpr size_t AUDIO_NUM_CHANNELS          = 4;    // 4-mic planar square array
constexpr size_t AUDIO_NUM_PAIRS             = 6;    // C(4, 2) = 6 unique baselines
constexpr size_t AUDIO_MAX_WINDOW_SIZE       = 2048; // Max DMA and DSP buffer capacity
constexpr size_t DEFAULT_WINDOW_SIZE         = 1024;
constexpr size_t AUDIO_DEFAULT_WINDOW_SIZE   = DEFAULT_WINDOW_SIZE;
constexpr uint32_t DEFAULT_SAMPLE_RATE       = 44100;
constexpr uint32_t AUDIO_DEFAULT_SAMPLE_RATE = DEFAULT_SAMPLE_RATE;
constexpr float PCM_INT32_TO_FLOAT_SCALE     = 2147483648.0f; // 2^31

// Supported Audio Configuration Whitelist
inline bool is_supported_audio_config(uint32_t sample_rate, uint32_t window_size) {
    return (sample_rate == 44100 || sample_rate == 48000) &&
           (window_size == 512 || window_size == 1024 || window_size == 2048);
}

// ==============================================================================
// 3. ACOUSTIC & CALIBRATION DEFAULTS
// ==============================================================================
constexpr float DEFAULT_MIC_DISTANCE_M               = 0.082f; // 82 mm physical spacing
constexpr float DEFAULT_CORRELATION_THRESHOLD        = 0.35f;
constexpr float DEFAULT_SOUND_LEVEL_THRESHOLD_DB_SPL = 55.0f;
constexpr float DEFAULT_DB_SPL_OFFSET                = 100.0f; // INMP441 dBFS-to-dB SPL offset
constexpr float DEFAULT_SNR_MARGIN_DB                = 8.0f;
constexpr float DEFAULT_ANGLE_OFFSET_DEG             = 0.0f;
constexpr bool  DEFAULT_ADAPTIVE_THRESHOLD_ENABLED   = true;

// Dual-Band GCC-PHAT Raised-Cosine Spectral Weighting Cutoffs
constexpr float GCC_PHAT_MIN_HZ      = 300.0f;
constexpr float GCC_PHAT_EDGE_MAX_HZ = 2200.0f;
constexpr float GCC_PHAT_DIAG_MAX_HZ = 1450.0f;
constexpr float GCC_PHAT_MAX_HZ      = GCC_PHAT_EDGE_MAX_HZ;
constexpr float GCC_PHAT_RAMP_HZ     = 100.0f;
constexpr float DOA_LOOP_CLOSURE_TOLERANCE_SAMPLES = 1.5f;

// Parameter Plausibility & Validation Bounds
constexpr float ANGLE_OFFSET_MIN_DEG    = -180.0f;
constexpr float ANGLE_OFFSET_MAX_DEG    = 180.0f;
constexpr float SNR_MARGIN_MIN_DB       = 0.0f;
constexpr float SNR_MARGIN_MAX_DB       = 60.0f;
constexpr float SPL_THRESHOLD_MIN_DB    = -200.0f;
constexpr float SPL_THRESHOLD_MAX_DB    = 200.0f;

// Environmental Defaults (Moist Air Thermodynamics Fallback)
constexpr float DEFAULT_TEMPERATURE_C   = 20.0f;
constexpr float DEFAULT_HUMIDITY_PCT    = 50.0f;
constexpr float DEFAULT_PRESSURE_PA     = 101325.0f;
constexpr float DEFAULT_SOUND_SPEED_MPS = 343.9f;
constexpr uint32_t SENSOR_READ_INTERVAL_MS = 2000;

// ==============================================================================
// 4. FREERTOS TASKS, QUEUES & WATCHDOG
// ==============================================================================
constexpr uint32_t TASK_WATCHDOG_TIMEOUT_SECONDS = 30;

// DSP Task (Core 1 - Real-Time Audio DSP)
constexpr uint32_t TASK_DSP_STACK_SIZE          = 8192;
constexpr unsigned TASK_DSP_PRIORITY            = 5;
constexpr int TASK_DSP_CORE                     = 1;
constexpr uint32_t DSP_DIAGNOSTIC_INTERVAL_MS   = 10000;

// Sensor & Display Task (Core 0 - UI & Sensors)
constexpr uint32_t TASK_SENSOR_DISPLAY_STACK_SIZE = 4096;
constexpr unsigned TASK_SENSOR_DISPLAY_PRIORITY   = 2;
constexpr int TASK_SENSOR_DISPLAY_CORE            = 0;
constexpr uint32_t DISPLAY_QUEUE_LENGTH           = 10;
constexpr uint32_t DISPLAY_QUEUE_TIMEOUT_MS       = 100;

// Network Task (Core 0 - REST Client & Telemetry)
constexpr uint32_t TASK_NETWORK_STACK_SIZE        = 8192;
constexpr unsigned TASK_NETWORK_PRIORITY          = 2;
constexpr int TASK_NETWORK_CORE                   = 0;
constexpr uint32_t NETWORK_QUEUE_LENGTH           = 64;
constexpr uint32_t NETWORK_BATCH_MAX_MEASUREMENTS = 8;
constexpr uint32_t NETWORK_BATCH_FLUSH_INTERVAL_MS = 200;
constexpr uint32_t NETWORK_MAX_REQUEST_INTERVAL_MS = 200;
constexpr int HTTP_RETRY_COUNT                    = 3;
constexpr uint32_t CONFIG_REFRESH_INTERVAL_MS     = 60000;

// Mutex Acquisition Timeouts
constexpr uint32_t CONFIG_MUTEX_FAST_TIMEOUT_MS   = 10;
constexpr uint32_t CONFIG_MUTEX_BLOCK_TIMEOUT_MS  = 100;
constexpr uint32_t DOA_MUTEX_BLOCK_TIMEOUT_MS     = 100;

// ==============================================================================
// 5. RADAR UI & DISPLAY COLOR PALETTE (RGB565)
// ==============================================================================
constexpr uint16_t COLOR_MUTED_GRAY     = 0x7BEF; // Divider lines and secondary text
constexpr uint16_t COLOR_RADAR_GRID     = 0x03E0; // Radar circular rings & crosshair axes
constexpr uint16_t COLOR_BADGE_BORDER   = 0x4208; // FIX mode badge outline

// Radar Geometry (360-degree circular scope)
constexpr int RADAR_CENTER_X            = 64;
constexpr int RADAR_CENTER_Y            = 44;
constexpr int RADAR_RADIUS              = 34;

// Needle Sound Pressure Level Color Thresholds
constexpr float NEEDLE_HIGH_SPL_THRESHOLD = 75.0f; // Red (> 75 dB SPL)
constexpr float NEEDLE_MED_SPL_THRESHOLD  = 60.0f; // Yellow (> 60 dB SPL)

// ==============================================================================
// 6. POWER-ON SELF-TEST (POST) HEURISTICS
// ==============================================================================
constexpr size_t POST_SAMPLE_COUNT          = 1024;
constexpr int POST_WARMUP_BLOCKS            = 6;
constexpr float POST_MIC_FLATLINE_RMS_DB    = -95.0f;
constexpr float POST_MIC_RAIL_SHORT_RMS_DB  = -3.0f;
constexpr size_t POST_MIC_MAX_COLLISION_PCT = 85;

// ==============================================================================
// 7. SETUP & CAPTIVE PORTAL
// ==============================================================================
constexpr uint32_t SETUP_BOOT_DELAY_MS  = 3000;
#define SETUP_AP_NAME                   "Acoustic-Locator-Setup"
#define SETUP_AP_PASSWORD               "configure123"
#define SETUP_AP_DEFAULT_IP_STR         "192.168.4.1"
