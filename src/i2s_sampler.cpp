#include <Arduino.h>
#include <driver/i2s.h>
#include <esp_log.h>
#include "app_config.h"

#define I2S_PORT (static_cast<i2s_port_t>(I2S_PORT_NUM))

static int32_t interleaved_buf[AUDIO_MAX_WINDOW_SIZE * 2];

bool init_i2s_microphones(uint32_t sample_rate) {
    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate = sample_rate,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT, // Stereo (oba mikrofona)
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 4,
        .dma_buf_len = 512,
        .use_apll = false,
        .tx_desc_auto_clear = false,
        .fixed_mclk = 0
    };

    i2s_pin_config_t pin_config = {
        .bck_io_num = I2S_BCLK_PIN,
        .ws_io_num = I2S_WS_PIN,
        .data_out_num = I2S_PIN_NO_CHANGE,
        .data_in_num = I2S_DIN_PIN
    };

    if (i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL) != ESP_OK) {
        return false;
    }
    if (i2s_set_pin(I2S_PORT, &pin_config) != ESP_OK) {
        i2s_driver_uninstall(I2S_PORT);
        return false;
    }
    i2s_zero_dma_buffer(I2S_PORT);

    // Let I2S clock run for ~140 ms so the INMP441 internal charge pump
    // and high-pass filter settle before delivering audio to consumers.
    const size_t warmup_bytes = POST_SAMPLE_COUNT * 2 * sizeof(int32_t);
    for (int b = 0; b < POST_WARMUP_BLOCKS; ++b) {
        size_t bytes_read = 0;
        i2s_read(I2S_PORT, interleaved_buf, warmup_bytes, &bytes_read, portMAX_DELAY);
    }
    return true;
}

void deinit_i2s_microphones() {
    i2s_driver_uninstall(I2S_PORT);
}

bool read_i2s_stereo(int32_t* left, int32_t* right, size_t count) {
    if (left == nullptr || right == nullptr || count == 0 ||
        count > AUDIO_MAX_WINDOW_SIZE) {
        return false;
    }

    size_t bytes_read = 0;
    const size_t bytes_requested = count * 2 * sizeof(int32_t);

    esp_err_t res = i2s_read(I2S_PORT, interleaved_buf, bytes_requested, &bytes_read, portMAX_DELAY);
    if (res != ESP_OK || bytes_read != bytes_requested) return false;

    // Razpleti stereo kanala: sodi vzorci Levi, lihi vzorci Desni
    for (size_t i = 0; i < count; i++) {
        left[i]  = interleaved_buf[2 * i];
        right[i] = interleaved_buf[2 * i + 1];
    }
    return true;
}