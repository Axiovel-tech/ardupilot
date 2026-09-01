#include "AP_Bootloader_config.h"

#if AP_BOOTLOADER_FLASH_FROM_SD_ENABLED

#include "abin_file.h"
#include "abin_header.h"
#include "ff.h"
#include "md5.h"

#include <AP_CheckFirmware/AP_CheckFirmware.h>
#include <AP_Math/crc.h>

#include <stddef.h>
#include <stdint.h>
#include <string.h>

namespace {

constexpr uint32_t IO_SIZE = 4096;
constexpr uint8_t MD5_SIZE = 16;
constexpr uint8_t DESCRIPTOR_SIGNATURE_SIZE = 8;

struct DescriptorLocation {
    uint32_t offset = 0;
    bool is_signed = false;
    uint32_t matches = 0;
};

uint8_t io_buffer[IO_SIZE + DESCRIPTOR_SIGNATURE_SIZE - 1U];

ABinValidationResult seek_to(FIL &file, uint32_t offset)
{
    return f_lseek(&file, offset) == FR_OK ?
           ABinValidationResult::VALID : ABinValidationResult::IO_ERROR;
}

ABinValidationResult read_exact(FIL &file, void *buffer, UINT wanted)
{
    UINT bytes_read = 0;
    if (f_read(&file, buffer, wanted, &bytes_read) != FR_OK) {
        return ABinValidationResult::IO_ERROR;
    }
    return bytes_read == wanted ?
           ABinValidationResult::VALID : ABinValidationResult::INVALID;
}

ABinValidationResult validate_md5(FIL &file, const ABinHeader &header)
{
    MD5Context context;
    MD5Init(&context);
    ABinValidationResult result = seek_to(file, header.body_offset);
    if (result != ABinValidationResult::VALID) {
        return result;
    }

    uint32_t remaining = header.body_size;
    while (remaining > 0) {
        const UINT wanted = remaining < IO_SIZE ? remaining : IO_SIZE;
        result = read_exact(file, io_buffer, wanted);
        if (result != ABinValidationResult::VALID) {
            return result;
        }
        MD5Update(&context, io_buffer, wanted);
        remaining -= wanted;
    }

    uint8_t calculated[MD5_SIZE];
    MD5Final(calculated, &context);
    return memcmp(calculated, header.expected_md5, sizeof(calculated)) == 0 ?
           ABinValidationResult::VALID : ABinValidationResult::INVALID;
}

void record_descriptor(const uint8_t *candidate, uint32_t offset, DescriptorLocation &location)
{
    const uint8_t unsigned_signature[] = AP_APP_DESCRIPTOR_SIGNATURE_UNSIGNED;
    const uint8_t signed_signature[] = AP_APP_DESCRIPTOR_SIGNATURE_SIGNED;
    bool is_signed;
    if (memcmp(candidate, unsigned_signature, sizeof(unsigned_signature)) == 0) {
        is_signed = false;
    } else if (memcmp(candidate, signed_signature, sizeof(signed_signature)) == 0) {
        is_signed = true;
    } else {
        return;
    }
    location.offset = offset;
    location.is_signed = is_signed;
    location.matches++;
}

ABinValidationResult find_descriptor(FIL &file, const ABinHeader &header,
                                     DescriptorLocation &location)
{
    ABinValidationResult result = seek_to(file, header.body_offset);
    if (result != ABinValidationResult::VALID) {
        return result;
    }

    uint32_t consumed = 0;
    uint8_t carry = 0;
    while (consumed < header.body_size) {
        const uint32_t remaining = header.body_size - consumed;
        const UINT wanted = remaining < IO_SIZE ? remaining : IO_SIZE;
        result = read_exact(file, &io_buffer[carry], wanted);
        if (result != ABinValidationResult::VALID) {
            return result;
        }

        const uint32_t available = carry + wanted;
        for (uint32_t index = 0; index + DESCRIPTOR_SIGNATURE_SIZE <= available; index++) {
            record_descriptor(&io_buffer[index], consumed - carry + index, location);
        }
        consumed += wanted;
        carry = available < DESCRIPTOR_SIGNATURE_SIZE - 1U ?
                available : DESCRIPTOR_SIGNATURE_SIZE - 1U;
        memmove(io_buffer, &io_buffer[available - carry], carry);
    }
    return location.matches == 1 ?
           ABinValidationResult::VALID : ABinValidationResult::INVALID;
}

ABinValidationResult crc_range(FIL &file, uint32_t offset, uint32_t length, uint32_t &crc)
{
    ABinValidationResult result = seek_to(file, offset);
    if (result != ABinValidationResult::VALID) {
        return result;
    }
    crc = 0;
    while (length > 0) {
        const UINT wanted = length < IO_SIZE ? length : IO_SIZE;
        result = read_exact(file, io_buffer, wanted);
        if (result != ABinValidationResult::VALID) {
            return result;
        }
        crc = crc32_small(crc, io_buffer, wanted);
        length -= wanted;
    }
    return ABinValidationResult::VALID;
}

template <typename Descriptor>
ABinValidationResult validate_descriptor(FIL &file, const ABinHeader &header,
                                         const DescriptorLocation &location,
                                         uint16_t board_id)
{
    if (header.body_size < sizeof(Descriptor) ||
        location.offset > header.body_size - sizeof(Descriptor)) {
        return ABinValidationResult::INVALID;
    }

    Descriptor descriptor;
    ABinValidationResult result = seek_to(file, header.body_offset + location.offset);
    if (result != ABinValidationResult::VALID) {
        return result;
    }
    result = read_exact(file, &descriptor, sizeof(descriptor));
    if (result != ABinValidationResult::VALID) {
        return result;
    }
    if (descriptor.image_size != header.body_size || descriptor.board_id != board_id) {
        return ABinValidationResult::INVALID;
    }

    const uint32_t crc1_length = location.offset + offsetof(Descriptor, image_crc1);
    const uint32_t crc2_offset = location.offset + offsetof(Descriptor, version_major);
    uint32_t crc1;
    uint32_t crc2;
    result = crc_range(file, header.body_offset, crc1_length, crc1);
    if (result != ABinValidationResult::VALID) {
        return result;
    }
    result = crc_range(file, header.body_offset + crc2_offset,
                       header.body_size - crc2_offset, crc2);
    if (result != ABinValidationResult::VALID) {
        return result;
    }
    return crc1 == descriptor.image_crc1 && crc2 == descriptor.image_crc2 ?
           ABinValidationResult::VALID : ABinValidationResult::INVALID;
}

ABinValidationResult validate_open_file(FIL &file, const ABinHeader &header,
                                        uint32_t maximum_image_size, uint16_t board_id)
{
    const uint32_t padded_size = (header.body_size + 127U) & ~127U;
    if (padded_size < header.body_size || padded_size > maximum_image_size) {
        return ABinValidationResult::INVALID;
    }

    ABinValidationResult result = validate_md5(file, header);
    if (result != ABinValidationResult::VALID) {
        return result;
    }
    DescriptorLocation location;
    result = find_descriptor(file, header, location);
    if (result != ABinValidationResult::VALID) {
        return result;
    }
    if (location.is_signed) {
        return validate_descriptor<app_descriptor_signed>(file, header, location, board_id);
    }
    return validate_descriptor<app_descriptor_unsigned>(file, header, location, board_id);
}

} // namespace

ABinValidationResult abin_validate(const char *path, uint32_t maximum_image_size,
                                   uint16_t board_id)
{
    FIL file;
    ABinHeader header;
    const ABinValidationResult open_result = abin_open_and_parse(path, file, header);
    if (open_result != ABinValidationResult::VALID) {
        return open_result;
    }

    const ABinValidationResult result =
        validate_open_file(file, header, maximum_image_size, board_id);
    f_close(&file);
    return result;
}

bool abin_stream_body(const char *path, ABinBodySink &sink)
{
    FIL file;
    ABinHeader header;
    if (abin_open_and_parse(path, file, header) != ABinValidationResult::VALID) {
        return false;
    }

    uint32_t remaining = header.body_size;
    bool success = true;
    while (success && remaining > 0) {
        const UINT wanted = remaining < IO_SIZE ? remaining : IO_SIZE;
        UINT bytes_read = 0;
        success = f_read(&file, io_buffer, wanted, &bytes_read) == FR_OK &&
                  bytes_read == wanted && sink.write(io_buffer, bytes_read);
        remaining -= bytes_read;
    }
    f_close(&file);
    return success && sink.finish();
}

#endif // AP_BOOTLOADER_FLASH_FROM_SD_ENABLED
