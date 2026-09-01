#pragma once

#include <stdint.h>

namespace HALSITL {

class SDCardOTA {
public:
    enum class Result : uint8_t {
        NO_UPDATE,
        FLASHED,
        FAILED,
        INTERRUPTED,
    };

    static bool enabled();
    static Result emulate();
};

} // namespace HALSITL
