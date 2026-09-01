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

bool SDCardOTA::enabled()
{
    const char *value = getenv("AXIO_SIM_SD_OTA");
    return value != nullptr && strcmp(value, "1") == 0;
}

SDCardOTA::Result SDCardOTA::emulate()
{
    const char *path = pending_update();
    if (path == nullptr) {
        return Result::NO_UPDATE;
    }

    if (requested_result("failure")) {
        const bool marked = move_file(path, FAILED_PATH);
        fprintf(stderr, "AXIO_SIM_SD_OTA: emulated update failure\n");
        return marked ? Result::FAILED : Result::INTERRUPTED;
    }

    if (strcmp(path, VERIFY_PATH) == 0) {
        if (!move_file(VERIFY_PATH, FLASH_PATH)) {
            return Result::INTERRUPTED;
        }
        path = FLASH_PATH;
    }

    if (requested_result("interrupted")) {
        fprintf(stderr, "AXIO_SIM_SD_OTA: emulated interrupted flash\n");
        return Result::INTERRUPTED;
    }

    if (!move_file(FLASH_PATH, FLASHED_PATH)) {
        return Result::INTERRUPTED;
    }
    fprintf(stderr, "AXIO_SIM_SD_OTA: emulated marker transition only, no STM32 flash occurred\n");
    return Result::FLASHED;
}

} // namespace HALSITL
