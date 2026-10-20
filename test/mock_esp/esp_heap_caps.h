#pragma once
#include <cstdlib>
#include <cstdint>

#define MALLOC_CAP_INTERNAL (1 << 0)
#define MALLOC_CAP_SPIRAM   (1 << 1)
#define MALLOC_CAP_8BIT     (1 << 2)

#ifdef __cplusplus
extern "C" {
#endif

void* heap_caps_malloc(size_t size, uint32_t caps);
void heap_caps_free(void* ptr);

#ifdef __cplusplus
}
#endif
