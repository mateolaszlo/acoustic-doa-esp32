#pragma once
#include <cstddef>
#include <cstdint>

#ifdef __cplusplus
extern "C" {
#endif

int dsps_fft2r_init_fc32(void* tables, int size);
void dsps_wind_hann_f32(float* window, int size);
int dsps_fft2r_fc32(float* data, int size);
int dsps_bit_rev_fc32(float* data, int size);

#ifdef __cplusplus
}
#endif
