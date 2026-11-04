#include "flash_from_sd.h"

#if AP_BOOTLOADER_FLASH_FROM_SD_ENABLED

#include "ch.h"
#include "ff.h"
#include "md5.h"
#include "stm32_util.h"
#include "support.h"

#include <AP_CheckFirmware/AP_CheckFirmware.h>
#include <AP_HAL_ChibiOS/sdcard.h>
#include <AP_HAL_ChibiOS/hwdef/common/flash.h>
#include <AP_Math/AP_Math.h>

#include <stdint.h>
#include <string.h>

namespace {

constexpr const char *INPUT_PATH = "/ardupilot.abin";
constexpr const char *VERIFY_PATH = "/ardupilot-verify.abin";
constexpr const char *FLASH_PATH = "/ardupilot-flash.abin";
constexpr const char *FLASHED_PATH = "/ardupilot-flashed.abin";
constexpr const char *FAILED_PATH = "/ardupilot-failed.abin";
constexpr uint32_t MAX_IO_SIZE = 4096;

const uint8_t *const APP_FLASH_BASE = reinterpret_cast<const uint8_t *>(
    0x08000000U + (FLASH_BOOTLOADER_LOAD_KB + APP_START_OFFSET_KB) * 1024U);

uint8_t parser_buffer[MAX_IO_SIZE];

enum class ABinValidationResult : uint8_t {
    VALID,
    INVALID,
    IO_ERROR,
};

bool decode_hex(char value, uint8_t &decoded)
{
    if (value >= '0' && value <= '9') {
        decoded = value - '0';
        return true;
    }
    if (value >= 'a' && value <= 'f') {
        decoded = value - 'a' + 10;
        return true;
    }
    if (value >= 'A' && value <= 'F') {
        decoded = value - 'A' + 10;
        return true;
    }
    return false;
}

bool is_body_delimiter(uint16_t index, UINT bytes_read)
{
    return bytes_read - index >= 3 &&
           memcmp(&parser_buffer[index], "--\n", 3) == 0;
}

bool file_exists(const char *path)
{
    FILINFO info;
    return f_stat(path, &info) == FR_OK;
}

const char *find_pending_update(bool &update_found)
{
    update_found = true;
    if (file_exists(FLASH_PATH)) {
        return FLASH_PATH;
    }
    if (file_exists(VERIFY_PATH)) {
        return VERIFY_PATH;
    }
    if (!file_exists(INPUT_PATH)) {
        update_found = false;
        return nullptr;
    }

    f_unlink(FLASHED_PATH);
    f_unlink(FAILED_PATH);
    return f_rename(INPUT_PATH, VERIFY_PATH) == FR_OK ? VERIFY_PATH : nullptr;
}

void mark_failed(const char *path)
{
    f_unlink(FAILED_PATH);
    f_rename(path, FAILED_PATH);
}

class ABinParser {
public:
    explicit ABinParser(const char *_path) : path(_path) {}
    virtual ~ABinParser() = default;

protected:
    virtual void name_value_callback(const char *name, const char *value) {}
    virtual void body_callback(const uint8_t *bytes, uint32_t nbytes) = 0;
    ABinValidationResult parse();

private:
    enum class State : uint8_t {
        START_NAME,
        ACCUMULATING_NAME,
        SKIPPING_POST_COLON_SPACES,
        START_VALUE,
        ACCUMULATING_VALUE,
        START_BODY,
        PROCESSING_BODY,
    };

    struct ParseContext {
        State state = State::START_NAME;
        uint16_t name_start = 0;
        uint16_t name_end = 0;
        uint16_t value_start = 0;
        bool body_started = false;
    };

    void emit_name_value(uint16_t name_start, uint16_t name_end,
                         uint16_t value_start, uint16_t value_end);
    bool process_byte(ParseContext &context, uint16_t &index, UINT bytes_read);
    bool process_chunk(ParseContext &context, UINT bytes_read);
    ABinValidationResult finish_parse(bool body_started,
                                      ABinValidationResult result);
    const char *path;
};

void ABinParser::emit_name_value(uint16_t name_start, uint16_t name_end,
                                 uint16_t value_start, uint16_t value_end)
{
    char name[80] {};
    char value[80] {};
    const uint16_t name_length = MIN(sizeof(name) - 1U, name_end - name_start);
    const uint16_t value_length = MIN(sizeof(value) - 1U, value_end - value_start);
    memcpy(name, &parser_buffer[name_start], name_length);
    memcpy(value, &parser_buffer[value_start], value_length);
    name_value_callback(name, value);
}

bool ABinParser::process_byte(ParseContext &context, uint16_t &index,
                              UINT bytes_read)
{
    switch (context.state) {
    case State::START_NAME:
        if (is_body_delimiter(index, bytes_read)) {
            index += 2;
            context.state = State::START_BODY;
            return true;
        }
        if (parser_buffer[index] == ':') {
            return false;
        }
        if (parser_buffer[index] != '\n') {
            context.name_start = index;
            context.state = State::ACCUMULATING_NAME;
        }
        return true;

    case State::ACCUMULATING_NAME:
        if (parser_buffer[index] == '\n') {
            return false;
        }
        if (parser_buffer[index] == ':') {
            context.name_end = index;
            context.state = State::SKIPPING_POST_COLON_SPACES;
        }
        return true;

    case State::SKIPPING_POST_COLON_SPACES:
        if (parser_buffer[index] == ' ') {
            return true;
        }
        context.state = State::START_VALUE;
        FALLTHROUGH;

    case State::START_VALUE:
        context.value_start = index;
        context.state = State::ACCUMULATING_VALUE;
        FALLTHROUGH;

    case State::ACCUMULATING_VALUE:
        if (parser_buffer[index] == '\n') {
            emit_name_value(context.name_start, context.name_end,
                            context.value_start, index);
            context.state = State::START_NAME;
        }
        return true;

    case State::START_BODY:
        context.body_started = true;
        context.state = State::PROCESSING_BODY;
        FALLTHROUGH;

    case State::PROCESSING_BODY:
        body_callback(&parser_buffer[index], bytes_read - index);
        index = bytes_read;
        return true;
    }
    return true;
}

bool ABinParser::process_chunk(ParseContext &context, UINT bytes_read)
{
    for (uint16_t index = 0; index < bytes_read; index++) {
        if (!process_byte(context, index, bytes_read)) {
            return false;
        }
    }
    return true;
}

ABinValidationResult ABinParser::finish_parse(bool body_started,
                                              ABinValidationResult result)
{
    if (!body_started) {
        return ABinValidationResult::INVALID;
    }
    if (result == ABinValidationResult::VALID) {
        body_callback(nullptr, 0);
    }
    return result;
}

ABinValidationResult ABinParser::parse()
{
    FIL file;
    if (f_open(&file, path, FA_READ) != FR_OK) {
        return ABinValidationResult::IO_ERROR;
    }

    ParseContext context;
    ABinValidationResult result = ABinValidationResult::VALID;

    while (true) {
        UINT bytes_read = 0;
        if (f_read(&file, parser_buffer, sizeof(parser_buffer), &bytes_read) != FR_OK) {
            result = ABinValidationResult::IO_ERROR;
            break;
        }
        if (bytes_read > sizeof(parser_buffer)) {
            result = ABinValidationResult::INVALID;
            break;
        }
        if (bytes_read == 0) {
            break;
        }

        if (!process_chunk(context, bytes_read)) {
            result = ABinValidationResult::INVALID;
            goto out;
        }
    }

    result = finish_parse(context.body_started, result);

out:
    f_close(&file);
    return result;
}

class ABinVerifier : public ABinParser {
public:
    using ABinParser::ABinParser;

    ABinValidationResult run()
    {
        MD5Init(&md5_context);
        const ABinValidationResult parse_result = parse();
        if (parse_result != ABinValidationResult::VALID) {
            return parse_result;
        }
        if (!has_expected_md5 || invalid_header) {
            return ABinValidationResult::INVALID;
        }

        uint8_t calculated_md5[16];
        MD5Final(calculated_md5, &md5_context);
        return memcmp(calculated_md5, expected_md5, sizeof(calculated_md5)) == 0 ?
               ABinValidationResult::VALID : ABinValidationResult::INVALID;
    }

protected:
    void name_value_callback(const char *name, const char *value) override
    {
        if (strcmp(name, "MD5") != 0) {
            return;
        }
        if (has_expected_md5 || strlen(value) != 32) {
            invalid_header = true;
            return;
        }
        for (uint8_t index = 0; index < sizeof(expected_md5); index++) {
            uint8_t high;
            uint8_t low;
            if (!decode_hex(value[index * 2U], high) ||
                !decode_hex(value[index * 2U + 1U], low)) {
                invalid_header = true;
                return;
            }
            expected_md5[index] = high << 4U | low;
        }
        has_expected_md5 = true;
    }

    void body_callback(const uint8_t *bytes, uint32_t nbytes) override
    {
        if (nbytes > 0) {
            MD5Update(&md5_context, bytes, nbytes);
        }
    }

private:
    uint8_t expected_md5[16] {};
    MD5Context md5_context;
    bool has_expected_md5 = false;
    bool invalid_header = false;
};

class ABinFlasher : public ABinParser {
public:
    using ABinParser::ABinParser;

    bool run()
    {
        for (uint8_t sector = 0; flash_func_sector_size(sector) != 0; sector++) {
            if (!flash_func_erase_sector(sector)) {
                return false;
            }
            led_toggle(LED_BOOTLOADER);
        }

        if (parse() != ABinValidationResult::VALID || failed) {
            return false;
        }

#if AP_CHECK_FIRMWARE_ENABLED
        return check_good_firmware() == check_fw_result_t::CHECK_FW_OK;
#else
        return true;
#endif
    }

protected:
    void body_callback(const uint8_t *bytes, uint32_t nbytes) override
    {
        if (failed) {
            return;
        }

        if (nbytes > 0) {
            memcpy(&buffer[buffer_offset], bytes, nbytes);
            buffer_offset += nbytes;
        }

        constexpr uint32_t WRITE_CHUNK_SIZE = 32U * 1024U;
        if (buffer_offset > WRITE_CHUNK_SIZE || nbytes == 0) {
            uint32_t consumed_size = WRITE_CHUNK_SIZE;
            uint32_t write_size = WRITE_CHUNK_SIZE;
            if (nbytes == 0) {
                consumed_size = buffer_offset;
                write_size = (consumed_size + 127U) & ~127U;
                memset(&buffer[consumed_size], 0, write_size - consumed_size);
            }
            if (write_size > board_info.fw_size - flash_offset ||
                !stm32_flash_write(
                    uint32_t(APP_FLASH_BASE) + flash_offset, buffer, write_size)) {
                failed = true;
                return;
            }
            flash_offset += write_size;
            buffer_offset -= consumed_size;
            memmove(buffer, &buffer[consumed_size], buffer_offset);
            led_toggle(LED_BOOTLOADER);
        }
    }

private:
    uint32_t flash_offset = 0;
    uint32_t buffer_offset = 0;
    uint8_t buffer[64U * 1024U] {};
    bool failed = false;
};

} // namespace

FlashFromSDResult flash_from_sd()
{
    peripheral_power_enable();
    if (!sdcard_init()) {
        return FlashFromSDResult::NO_UPDATE;
    }

    FlashFromSDResult result = FlashFromSDResult::NO_UPDATE;
    ABinVerifier *verifier = nullptr;
    ABinFlasher *flasher = nullptr;
    ABinValidationResult validation = ABinValidationResult::IO_ERROR;
    bool update_found = false;
    const char *path = find_pending_update(update_found);
    if (path == nullptr) {
        if (update_found) {
            result = FlashFromSDResult::FAILED;
        }
        goto out;
    }
    result = FlashFromSDResult::FAILED;

    verifier = NEW_NOTHROW ABinVerifier{path};
    if (verifier == nullptr) {
        goto out;
    }
    validation = verifier->run();
    delete verifier;
    verifier = nullptr;
    if (validation != ABinValidationResult::VALID) {
        if (validation == ABinValidationResult::INVALID &&
            strcmp(path, VERIFY_PATH) == 0) {
            mark_failed(path);
        }
        goto out;
    }

    if (strcmp(path, VERIFY_PATH) == 0) {
        if (f_rename(VERIFY_PATH, FLASH_PATH) != FR_OK) {
            goto out;
        }
        path = FLASH_PATH;
    }

    flasher = NEW_NOTHROW ABinFlasher{path};
    if (flasher == nullptr || !flasher->run()) {
        goto out;
    }
    delete flasher;
    flasher = nullptr;

    f_unlink(FLASHED_PATH);
    if (f_rename(FLASH_PATH, FLASHED_PATH) == FR_OK) {
        result = FlashFromSDResult::FLASHED;
    }

out:
    delete verifier;
    delete flasher;
    sdcard_stop();
    return result;
}

#endif // AP_BOOTLOADER_FLASH_FROM_SD_ENABLED
