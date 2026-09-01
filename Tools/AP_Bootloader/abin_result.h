#pragma once

#include <stdint.h>

enum class ABinValidationResult : uint8_t {
    VALID,
    INVALID,
    IO_ERROR,
};
