#include "flash_from_sd.h"

#if AP_BOOTLOADER_FLASH_FROM_SD_ENABLED

#include "abin_file.h"
#include "ch.h"
#include "ff.h"
#include "stm32_util.h"
#include "support.h"

#include <AP_CheckFirmware/AP_CheckFirmware.h>
#include <AP_HAL_ChibiOS/hwdef/common/flash.h>

#include <stdint.h>
#include <string.h>

namespace {

constexpr const char *INPUT_PATH = "/ardupilot.abin";
constexpr const char *VERIFY_PATH = "/ardupilot-verify.abin";
constexpr const char *FLASH_PATH = "/ardupilot-flash.abin";
constexpr const char *FLASHED_PATH = "/ardupilot-flashed.abin";
constexpr const char *FAILED_PATH = "/ardupilot-failed.abin";

static const uint32_t APP_FLASH_BASE =
    0x08000000U + (FLASH_BOOTLOADER_LOAD_KB + APP_START_OFFSET_KB) * 1024U;

bool file_exists(const char *path)
{
    FILINFO info;
    return f_stat(path, &info) == FR_OK;
}

const char *find_pending_update()
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

    f_unlink(FLASHED_PATH);
    f_unlink(FAILED_PATH);
    if (f_rename(INPUT_PATH, VERIFY_PATH) != FR_OK) {
        return nullptr;
    }
    return VERIFY_PATH;
}

void mark_failed(const char *path)
{
    f_unlink(FAILED_PATH);
    f_rename(path, FAILED_PATH);
}

class ABinFlasher : public ABinBodySink {
public:
    enum class Result : uint8_t {
        SUCCESS,
        WRITE_FAILED,
        INVALID_FIRMWARE,
    };

    Result run(const char *path)
    {
        for (uint8_t sector = 0; flash_func_sector_size(sector) != 0; sector++) {
            if (!flash_func_erase_sector(sector)) {
                return Result::WRITE_FAILED;
            }
            led_toggle(LED_BOOTLOADER);
        }

        if (!abin_stream_body(path, *this) || failed) {
            return Result::WRITE_FAILED;
        }

#if AP_CHECK_FIRMWARE_ENABLED
        if (check_good_firmware() != check_fw_result_t::CHECK_FW_OK) {
            return Result::INVALID_FIRMWARE;
        }
#endif
        return Result::SUCCESS;
    }

    bool write(const uint8_t *bytes, uint32_t nbytes) override
    {
        if (nbytes > sizeof(buffer) - buffer_offset) {
            failed = true;
            return false;
        }

        memcpy(&buffer[buffer_offset], bytes, nbytes);
        buffer_offset += nbytes;
        while (buffer_offset >= WRITE_CHUNK_SIZE) {
            if (!write_chunk(WRITE_CHUNK_SIZE, WRITE_CHUNK_SIZE)) {
                return false;
            }
        }
        return true;
    }

    bool finish() override
    {
        if (buffer_offset == 0) {
            return !failed;
        }
        const uint32_t padded_size = (buffer_offset + FLASH_WRITE_ALIGNMENT - 1U) &
                                     ~(FLASH_WRITE_ALIGNMENT - 1U);
        const uint32_t consumed_size = buffer_offset;
        memset(&buffer[buffer_offset], 0, padded_size - buffer_offset);
        return write_chunk(padded_size, consumed_size);
    }

private:
    static constexpr uint32_t WRITE_CHUNK_SIZE = 32U * 1024U;
    static constexpr uint32_t FLASH_WRITE_ALIGNMENT = 128U;

    bool write_chunk(uint32_t write_size, uint32_t consumed_size)
    {
        if (!stm32_flash_write(APP_FLASH_BASE + flash_offset, buffer, write_size)) {
            failed = true;
            return false;
        }
        flash_offset += write_size;
        buffer_offset -= consumed_size;
        memmove(buffer, &buffer[consumed_size], buffer_offset);
        led_toggle(LED_BOOTLOADER);
        return true;
    }

    uint32_t flash_offset = 0;
    uint32_t buffer_offset = 0;
    uint8_t buffer[64U * 1024U] {};
    bool failed = false;
};

} // namespace

bool flash_from_sd()
{
    peripheral_power_enable();
    if (!sdcard_init()) {
        return false;
    }

    bool success = false;
    ABinFlasher *flasher = nullptr;
    ABinFlasher::Result flash_result = ABinFlasher::Result::WRITE_FAILED;
    const char *path = find_pending_update();
    if (path == nullptr) {
        goto out;
    }

    if (!abin_validate(path, board_info.fw_size, APJ_BOARD_ID)) {
        mark_failed(path);
        goto out;
    }

    if (strcmp(path, VERIFY_PATH) == 0) {
        if (f_rename(VERIFY_PATH, FLASH_PATH) != FR_OK) {
            goto out;
        }
        path = FLASH_PATH;
    }

    flasher = NEW_NOTHROW ABinFlasher;
    if (flasher == nullptr) {
        goto out;
    }
    flash_result = flasher->run(path);
    delete flasher;
    flasher = nullptr;
    if (flash_result == ABinFlasher::Result::INVALID_FIRMWARE) {
        mark_failed(path);
        goto out;
    }
    if (flash_result != ABinFlasher::Result::SUCCESS) {
        goto out;
    }

    f_unlink(FLASHED_PATH);
    success = f_rename(FLASH_PATH, FLASHED_PATH) == FR_OK;

out:
    delete flasher;
    sdcard_stop();
    return success;
}

#endif // AP_BOOTLOADER_FLASH_FROM_SD_ENABLED
