#pragma once

#include <stdint.h>
#include <stdio.h>

using FRESULT = uint8_t;
using FSIZE_t = uint64_t;
using UINT = unsigned int;

constexpr FRESULT FR_OK = 0;
constexpr FRESULT FR_DISK_ERR = 1;
constexpr uint8_t FA_READ = 1;

struct FIL {
    FILE *handle = nullptr;
    FSIZE_t size = 0;
};

static inline FRESULT f_open(FIL *file, const char *path, uint8_t)
{
    file->handle = fopen(path, "rb");
    if (file->handle == nullptr || fseek(file->handle, 0, SEEK_END) != 0) {
        return FR_DISK_ERR;
    }
    const long size = ftell(file->handle);
    if (size < 0 || fseek(file->handle, 0, SEEK_SET) != 0) {
        fclose(file->handle);
        file->handle = nullptr;
        return FR_DISK_ERR;
    }
    file->size = static_cast<FSIZE_t>(size);
    return FR_OK;
}

static inline FRESULT f_read(FIL *file, void *buffer, UINT wanted, UINT *read)
{
    *read = static_cast<UINT>(fread(buffer, 1, wanted, file->handle));
    return ferror(file->handle) == 0 ? FR_OK : FR_DISK_ERR;
}

static inline FRESULT f_lseek(FIL *file, FSIZE_t offset)
{
    return fseek(file->handle, static_cast<long>(offset), SEEK_SET) == 0 ? FR_OK : FR_DISK_ERR;
}

static inline FSIZE_t f_tell(FIL *file)
{
    return static_cast<FSIZE_t>(ftell(file->handle));
}

static inline FSIZE_t f_size(FIL *file)
{
    return file->size;
}

static inline FRESULT f_close(FIL *file)
{
    const FRESULT result = fclose(file->handle) == 0 ? FR_OK : FR_DISK_ERR;
    file->handle = nullptr;
    return result;
}
