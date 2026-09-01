#pragma once

#include "ff.h"

#include <stdint.h>

struct ABinHeader {
    uint8_t expected_md5[16] {};
    uint32_t body_offset = 0;
    uint32_t body_size = 0;
    bool has_md5 = false;
};

bool abin_open_and_parse(const char *path, FIL &file, ABinHeader &header);
