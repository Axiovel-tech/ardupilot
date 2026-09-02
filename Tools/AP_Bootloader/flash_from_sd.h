#pragma once

#include "AP_Bootloader_config.h"

#include <stdint.h>

enum class FlashFromSDResult : uint8_t {
    NO_UPDATE,
    FLASHED,
    // An update was found but did not complete. The caller must validate the
    // installed application before deciding whether it is safe to boot.
    FAILED,
};

constexpr bool boot_after_sd_update(FlashFromSDResult result, bool firmware_ok)
{
    return result == FlashFromSDResult::FLASHED ||
           (result == FlashFromSDResult::FAILED && firmware_ok);
}

static_assert(!boot_after_sd_update(FlashFromSDResult::NO_UPDATE, true),
              "an explicit bootloader hold must remain active without an SD update");
static_assert(!boot_after_sd_update(FlashFromSDResult::NO_UPDATE, false),
              "an explicit bootloader hold must not depend on firmware state");
static_assert(boot_after_sd_update(FlashFromSDResult::FLASHED, false),
              "a completed SD update must boot");
static_assert(boot_after_sd_update(FlashFromSDResult::FLASHED, true),
              "a completed SD update must not depend on prior firmware state");
static_assert(!boot_after_sd_update(FlashFromSDResult::FAILED, false),
              "a failed SD update must not boot invalid firmware");
static_assert(boot_after_sd_update(FlashFromSDResult::FAILED, true),
              "a failed SD update may return to validated firmware");

#if AP_BOOTLOADER_FLASH_FROM_SD_ENABLED

FlashFromSDResult flash_from_sd();

#endif  // AP_BOOTLOADER_FLASH_FROM_SD_ENABLED
