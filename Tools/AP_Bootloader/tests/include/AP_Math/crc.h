#pragma once

#include <stdint.h>

static inline uint32_t crc32_small(uint32_t crc, const uint8_t *buffer, uint32_t size)
{
    while (size-- > 0) {
        crc ^= *buffer++;
        for (uint8_t bit = 0; bit < 8; bit++) {
            const uint32_t mask = -(crc & 1U);
            crc = (crc >> 1U) ^ (0xEDB88320U & mask);
        }
    }
    return crc;
}
