#pragma once
#include <cstdint>
#include <cstddef>

/**
 * @brief Initializes the synchronized dual-I2S 4-microphone capture subsystem:
 *        - I2S0 (Master RX) on BCLK (GPIO 4), WS (GPIO 5), DIN0 (GPIO 6) for Mics 0 & 1.
 *        - I2S1 (Slave RX) on DIN1 (GPIO 7) sharing BCLK/WS internally via the ESP32-S3
 *          GPIO Matrix for Mics 2 & 3, armed in Slave-before-Master startup order.
 */
bool init_i2s_microphones(uint32_t sample_rate);

/**
 * @brief Stops and uninstalls both I2S0 and I2S1 drivers.
 */
void deinit_i2s_microphones();

/**
 * @brief Reads a synchronized block of 4-channel 32-bit PCM samples from I2S0 and I2S1.
 *
 * @param ch0 Output buffer for Mic 0 (I2S0 Left, Right-Back)
 * @param ch1 Output buffer for Mic 1 (I2S0 Right, Right-Front)
 * @param ch2 Output buffer for Mic 2 (I2S1 Left, Left-Front)
 * @param ch3 Output buffer for Mic 3 (I2S1 Right, Left-Back)
 * @param count Number of samples per channel (1 .. AUDIO_MAX_WINDOW_SIZE)
 * @return true on successful capture of all 4 channels, false otherwise.
 */
bool read_i2s_quad(int32_t* ch0, int32_t* ch1, int32_t* ch2, int32_t* ch3, size_t count);

/**
 * @brief Legacy 2-channel stereo wrapper (reads Bus 0 into left/right and drains Bus 1).
 */
bool read_i2s_stereo(int32_t* left, int32_t* right, size_t count);
