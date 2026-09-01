#include "AP_Bootloader_config.h"

#if AP_BOOTLOADER_FLASH_FROM_SD_ENABLED

#include "abin_header.h"
#include "md5.h"

#include <stdint.h>
#include <string.h>

namespace {

constexpr uint16_t MAX_HEADER_SIZE = 1024;
constexpr uint8_t MAX_HEADER_LINE_SIZE = 96;
constexpr uint8_t MD5_TEXT_SIZE = 32;
constexpr uint8_t MD5_SIZE = 16;

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

bool parse_md5(const char *text, uint8_t *md5)
{
    if (strlen(text) != MD5_TEXT_SIZE) {
        return false;
    }
    for (uint8_t index = 0; index < MD5_SIZE; index++) {
        uint8_t high;
        uint8_t low;
        if (!decode_hex(text[index * 2U], high) || !decode_hex(text[index * 2U + 1U], low)) {
            return false;
        }
        md5[index] = high << 4U | low;
    }
    return true;
}

bool read_header_line(FIL &file, char *line, uint8_t line_capacity,
                      uint16_t &header_size)
{
    uint8_t line_length = 0;
    while (header_size < MAX_HEADER_SIZE) {
        UINT bytes_read = 0;
        char value;
        if (f_read(&file, &value, 1, &bytes_read) != FR_OK || bytes_read != 1) {
            return false;
        }
        header_size++;
        if (value == '\n') {
            line[line_length] = '\0';
            return true;
        }
        if (line_length >= line_capacity - 1U) {
            return false;
        }
        if (value != '\r') {
            line[line_length++] = value;
        }
    }
    return false;
}

bool set_body_bounds(FIL &file, ABinHeader &header)
{
    header.body_offset = f_tell(&file);
    const FSIZE_t file_size = f_size(&file);
    if (file_size < header.body_offset) {
        return false;
    }
    const FSIZE_t body_size = file_size - header.body_offset;
    if (body_size == 0 || body_size > UINT32_MAX) {
        return false;
    }
    header.body_size = body_size;
    return header.has_md5;
}

bool accept_header_line(const char *line, ABinHeader &header)
{
    if (strncmp(line, "MD5: ", 5) != 0) {
        return true;
    }
    if (header.has_md5 || !parse_md5(&line[5], header.expected_md5)) {
        return false;
    }
    header.has_md5 = true;
    return true;
}

bool parse_header(FIL &file, ABinHeader &header)
{
    char line[MAX_HEADER_LINE_SIZE] {};
    uint16_t header_size = 0;
    while (read_header_line(file, line, sizeof(line), header_size)) {
        if (strcmp(line, "--") == 0) {
            return set_body_bounds(file, header);
        }
        if (!accept_header_line(line, header)) {
            return false;
        }
    }
    return false;
}

} // namespace

bool abin_open_and_parse(const char *path, FIL &file, ABinHeader &header)
{
    if (f_open(&file, path, FA_READ) != FR_OK) {
        return false;
    }
    if (!parse_header(file, header)) {
        f_close(&file);
        return false;
    }
    return true;
}

#endif // AP_BOOTLOADER_FLASH_FROM_SD_ENABLED
