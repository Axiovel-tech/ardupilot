#pragma once

#include <stdint.h>

#define AP_APP_DESCRIPTOR_SIGNATURE_SIGNED   { 0x41, 0xa3, 0xe5, 0xf2, 0x65, 0x69, 0x92, 0x07 }
#define AP_APP_DESCRIPTOR_SIGNATURE_UNSIGNED { 0x40, 0xa2, 0xe4, 0xf1, 0x64, 0x68, 0x91, 0x06 }

struct app_descriptor_unsigned {
    uint8_t sig[8];
    uint32_t image_crc1;
    uint32_t image_crc2;
    uint32_t image_size;
    uint32_t git_hash;
    uint8_t version_major;
    uint8_t version_minor;
    uint16_t board_id;
    uint8_t reserved[8];
};

struct app_descriptor_signed {
    uint8_t sig[8];
    uint32_t image_crc1;
    uint32_t image_crc2;
    uint32_t image_size;
    uint32_t git_hash;
    uint32_t signature_length;
    uint8_t signature[72];
    uint8_t version_major;
    uint8_t version_minor;
    uint16_t board_id;
    uint8_t reserved[8];
};

static_assert(sizeof(app_descriptor_unsigned) == 36, "unsigned descriptor layout changed");
static_assert(sizeof(app_descriptor_signed) == 112, "signed descriptor layout changed");
