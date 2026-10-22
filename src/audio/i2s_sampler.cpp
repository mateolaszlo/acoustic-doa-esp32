#include "audio/i2s_sampler.h"
#include <Arduino.h>
#include <driver/i2s.h>
#include <driver/gpio.h>
#include <soc/gpio_sig_map.h>
#include <esp_rom_gpio.h>
#include <esp_log.h>
#include "config/app_config.h"

#define I2S_MASTER_PORT (static_cast<i2s_port_t>(I2S_MASTER_PORT_NUM))
#define I2S_SLAVE_PORT  (static_cast<i2s_port_t>(I2S_SLAVE_PORT_NUM))

static const char* TAG = "I2S_SAMPLER";
static int32_t interleaved_buf[AUDIO_MAX_WINDOW_SIZE * 2];

bool init_i2s_microphones(uint32_t sample_rate) {
    i2s_config_t master_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate = sample_rate,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT, // Stereo (Mic 0 Left, Mic 1 Right)
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 4,
        .dma_buf_len = 512,
        .use_apll = false,
        .tx_desc_auto_clear = false,
        .fixed_mclk = 0
    };

    i2s_config_t slave_config = master_config;
    slave_config.mode = (i2s_mode_t)(I2S_MODE_SLAVE | I2S_MODE_RX);

    i2s_pin_config_t master_pins = {
        .mck_io_num = I2S_PIN_NO_CHANGE,
        .bck_io_num = I2S_BCLK_PIN,
        .ws_io_num = I2S_WS_PIN,
        .data_out_num = I2S_PIN_NO_CHANGE,
        .data_in_num = I2S_DIN0_PIN
    };

    // Pass I2S_PIN_NO_CHANGE for BCLK/WS on Slave so i2s_set_pin() does not
    // reconfigure GPIO 5/4 as input-only and disable I2S0's clock output drivers.
    i2s_pin_config_t slave_pins = {
        .mck_io_num = I2S_PIN_NO_CHANGE,
        .bck_io_num = I2S_PIN_NO_CHANGE,
        .ws_io_num = I2S_PIN_NO_CHANGE,
        .data_out_num = I2S_PIN_NO_CHANGE,
        .data_in_num = I2S_DIN1_PIN
    };

    if (i2s_driver_install(I2S_MASTER_PORT, &master_config, 0, NULL) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install I2S0 Master driver");
        return false;
    }
    if (i2s_driver_install(I2S_SLAVE_PORT, &slave_config, 0, NULL) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install I2S1 Slave driver");
        i2s_driver_uninstall(I2S_MASTER_PORT);
        return false;
    }

    // Pause both peripherals before configuring pins and arming DMA
    i2s_stop(I2S_MASTER_PORT);
    i2s_stop(I2S_SLAVE_PORT);

    if (i2s_set_pin(I2S_MASTER_PORT, &master_pins) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set I2S0 Master pins");
        i2s_driver_uninstall(I2S_SLAVE_PORT);
        i2s_driver_uninstall(I2S_MASTER_PORT);
        return false;
    }
    if (i2s_set_pin(I2S_SLAVE_PORT, &slave_pins) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set I2S1 Slave pins");
        i2s_driver_uninstall(I2S_SLAVE_PORT);
        i2s_driver_uninstall(I2S_MASTER_PORT);
        return false;
    }

    // Route GPIO 5 (BCLK) and GPIO 4 (WS) bidirectionally through the ESP32-S3
    // GPIO Matrix so I2S0 drives the external clock pads while I2S1 receives
    // the exact same BCLK/WS edges internally without external jumper wires.
    gpio_set_direction(static_cast<gpio_num_t>(I2S_BCLK_PIN), GPIO_MODE_INPUT_OUTPUT);
    esp_rom_gpio_connect_out_signal(I2S_BCLK_PIN, I2S0I_BCK_OUT_IDX, false, false);
    esp_rom_gpio_connect_in_signal(I2S_BCLK_PIN, I2S1I_BCK_IN_IDX, false);

    gpio_set_direction(static_cast<gpio_num_t>(I2S_WS_PIN), GPIO_MODE_INPUT_OUTPUT);
    esp_rom_gpio_connect_out_signal(I2S_WS_PIN, I2S0I_WS_OUT_IDX, false, false);
    esp_rom_gpio_connect_in_signal(I2S_WS_PIN, I2S1I_WS_IN_IDX, false);

    // Deterministic Slave-before-Master startup sequence:
    // Clear both DMA rings, arm I2S1 Slave RX first, then start I2S0 Master RX
    // so both controllers latch on the exact same WS frame edge.
    i2s_zero_dma_buffer(I2S_SLAVE_PORT);
    i2s_zero_dma_buffer(I2S_MASTER_PORT);

    if (i2s_start(I2S_SLAVE_PORT) != ESP_OK ||
        i2s_start(I2S_MASTER_PORT) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start I2S1/I2S0 controllers in lockstep");
        i2s_driver_uninstall(I2S_SLAVE_PORT);
        i2s_driver_uninstall(I2S_MASTER_PORT);
        return false;
    }

    // Let I2S clock run for ~140 ms (6 blocks) in lockstep across both ports
    // so all 4 INMP441 charge pumps and high-pass filters settle.
    static_assert(POST_SAMPLE_COUNT <= AUDIO_MAX_WINDOW_SIZE,
                  "POST_SAMPLE_COUNT exceeds interleaved_buf capacity");
    const size_t warmup_bytes = POST_SAMPLE_COUNT * 2 * sizeof(int32_t);
    for (int b = 0; b < POST_WARMUP_BLOCKS; ++b) {
        size_t bytes_read_0 = 0;
        size_t bytes_read_1 = 0;
        esp_err_t res0 = i2s_read(I2S_MASTER_PORT, interleaved_buf, warmup_bytes, &bytes_read_0, portMAX_DELAY);
        esp_err_t res1 = i2s_read(I2S_SLAVE_PORT, interleaved_buf, warmup_bytes, &bytes_read_1, portMAX_DELAY);
        if (res0 != ESP_OK || res1 != ESP_OK ||
            bytes_read_0 != warmup_bytes || bytes_read_1 != warmup_bytes) {
            ESP_LOGE(TAG, "Failed I2S lockstep warmup read on block %d", b);
            i2s_driver_uninstall(I2S_SLAVE_PORT);
            i2s_driver_uninstall(I2S_MASTER_PORT);
            return false;
        }
    }
    return true;
}

static void resync_i2s_ports() {
    i2s_stop(I2S_MASTER_PORT);
    i2s_stop(I2S_SLAVE_PORT);
    i2s_zero_dma_buffer(I2S_SLAVE_PORT);
    i2s_zero_dma_buffer(I2S_MASTER_PORT);
    i2s_start(I2S_SLAVE_PORT);
    i2s_start(I2S_MASTER_PORT);
}

void deinit_i2s_microphones() {
    i2s_driver_uninstall(I2S_SLAVE_PORT);
    i2s_driver_uninstall(I2S_MASTER_PORT);
}

bool read_i2s_quad(int32_t* ch0, int32_t* ch1, int32_t* ch2, int32_t* ch3, size_t count) {
    if (ch0 == nullptr || ch1 == nullptr || ch2 == nullptr || ch3 == nullptr ||
        count == 0 || count > AUDIO_MAX_WINDOW_SIZE) {
        return false;
    }

    const size_t bytes_requested = count * 2 * sizeof(int32_t);
    size_t bytes_read_0 = 0;
    size_t bytes_read_1 = 0;

    // 1. Read and demux Bus 0 (I2S0 Master RX -> Mic 0 Left, Mic 1 Right)
    esp_err_t res0 = i2s_read(I2S_MASTER_PORT, interleaved_buf, bytes_requested, &bytes_read_0, portMAX_DELAY);
    if (res0 != ESP_OK || bytes_read_0 != bytes_requested) {
        resync_i2s_ports();
        return false;
    }
    for (size_t i = 0; i < count; i++) {
        ch0[i] = interleaved_buf[2 * i];
        ch1[i] = interleaved_buf[2 * i + 1];
    }

    // 2. Read and demux Bus 1 (I2S1 Slave RX -> Mic 2 Left, Mic 3 Right)
    esp_err_t res1 = i2s_read(I2S_SLAVE_PORT, interleaved_buf, bytes_requested, &bytes_read_1, portMAX_DELAY);
    if (res1 != ESP_OK || bytes_read_1 != bytes_requested) {
        resync_i2s_ports();
        return false;
    }
    for (size_t i = 0; i < count; i++) {
        ch2[i] = interleaved_buf[2 * i];
        ch3[i] = interleaved_buf[2 * i + 1];
    }

    return true;
}

bool read_i2s_stereo(int32_t* left, int32_t* right, size_t count) {
    if (left == nullptr || right == nullptr || count == 0 ||
        count > AUDIO_MAX_WINDOW_SIZE) {
        return false;
    }

    size_t bytes_read_0 = 0;
    size_t bytes_read_1 = 0;
    const size_t bytes_requested = count * 2 * sizeof(int32_t);

    esp_err_t res0 = i2s_read(I2S_MASTER_PORT, interleaved_buf, bytes_requested, &bytes_read_0, portMAX_DELAY);
    if (res0 != ESP_OK || bytes_read_0 != bytes_requested) {
        resync_i2s_ports();
        return false;
    }

    // Demux stereo Bus 0: even indices Left (Mic 0), odd indices Right (Mic 1)
    for (size_t i = 0; i < count; i++) {
        left[i]  = interleaved_buf[2 * i];
        right[i] = interleaved_buf[2 * i + 1];
    }

    // Drain Slave RX DMA buffer in lockstep so Bus 1 does not stall or drift
    esp_err_t res1 = i2s_read(I2S_SLAVE_PORT, interleaved_buf, bytes_requested, &bytes_read_1, portMAX_DELAY);
    if (res1 != ESP_OK || bytes_read_1 != bytes_requested) {
        resync_i2s_ports();
        return false;
    }
    return true;
}