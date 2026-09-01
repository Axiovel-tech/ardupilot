#pragma once

#include "abin_result.h"

#include <stdint.h>

class ABinBodySink {
public:
    virtual ~ABinBodySink() = default;
    virtual bool write(const uint8_t *bytes, uint32_t nbytes) = 0;
    virtual bool finish() = 0;
};

ABinValidationResult abin_validate(const char *path, uint32_t maximum_image_size,
                                   uint16_t board_id);
bool abin_stream_body(const char *path, ABinBodySink &sink);
