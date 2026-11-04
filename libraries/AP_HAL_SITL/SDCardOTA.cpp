#include "SDCardOTA.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

namespace HALSITL {

namespace {

constexpr const char *INPUT_PATH = "ardupilot.abin";
constexpr const char *VERIFY_PATH = "ardupilot-verify.abin";
constexpr const char *FLASH_PATH = "ardupilot-flash.abin";
constexpr const char *FLASHED_PATH = "ardupilot-flashed.abin";
constexpr const char *FAILED_PATH = "ardupilot-failed.abin";

bool file_exists(const char *path)
{
    return access(path, F_OK) == 0;
}

bool move_file(const char *source, const char *destination)
{
    unlink(destination);
    return rename(source, destination) == 0;
}

const char *pending_update()
{
    if (file_exists(FLASH_PATH)) {
        return FLASH_PATH;
    }
    if (file_exists(VERIFY_PATH)) {
        return VERIFY_PATH;
    }
    if (!file_exists(INPUT_PATH)) {
        return nullptr;
    }

    unlink(FLASHED_PATH);
    unlink(FAILED_PATH);
    return move_file(INPUT_PATH, VERIFY_PATH) ? VERIFY_PATH : nullptr;
}

bool requested_result(const char *value)
{
    const char *configured = getenv("AXIO_SIM_SD_OTA_RESULT");
    return configured != nullptr && strcmp(configured, value) == 0;
}

} // namespace

bool sdcard_ota_enabled()
{
    const char *value = getenv("AXIO_SIM_SD_OTA");
    return value != nullptr && strcmp(value, "1") == 0;
}

void emulate_sdcard_ota()
{
    const char *path = pending_update();
    if (path == nullptr) {
        return;
    }

    if (requested_result("failure")) {
        if (strcmp(path, VERIFY_PATH) == 0 && move_file(path, FAILED_PATH)) {
            fprintf(stderr, "AXIO_SIM_SD_OTA: emulated update rejection\n");
        }
        return;
    }

    if (strcmp(path, VERIFY_PATH) == 0) {
        if (!move_file(VERIFY_PATH, FLASH_PATH)) {
            return;
        }
        path = FLASH_PATH;
    }

    if (requested_result("interrupted")) {
        fprintf(stderr, "AXIO_SIM_SD_OTA: emulated interrupted flash\n");
        return;
    }

    if (move_file(FLASH_PATH, FLASHED_PATH)) {
        fprintf(stderr, "AXIO_SIM_SD_OTA: emulated marker transition only, no STM32 flash occurred\n");
    }
}

} // namespace HALSITL
