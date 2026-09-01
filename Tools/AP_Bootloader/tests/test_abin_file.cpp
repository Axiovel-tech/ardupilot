#include "abin_file.h"
#include "md5.h"

#include <AP_CheckFirmware/AP_CheckFirmware.h>
#include <AP_Math/crc.h>

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include <fstream>
#include <string>
#include <vector>

namespace {

constexpr uint16_t BOARD_ID = 1177;
constexpr uint32_t IMAGE_SIZE_LIMIT = 64U * 1024U;

int failures;

#define CHECK(expression) check((expression), #expression, __LINE__)

void check(bool passed, const char *expression, int line)
{
    if (!passed) {
        fprintf(stderr, "line %d: check failed: %s\n", line, expression);
        failures++;
    }
}

class TemporaryFile {
public:
    TemporaryFile(const std::string &header, const std::vector<uint8_t> &body)
    {
        char path_template[] = "/tmp/abin-test-XXXXXX";
        const int descriptor = mkstemp(path_template);
        CHECK(descriptor >= 0);
        if (descriptor >= 0) {
            close(descriptor);
            path = path_template;
            std::ofstream output(path, std::ios::binary | std::ios::trunc);
            output.write(header.data(), header.size());
            output.write(reinterpret_cast<const char *>(body.data()), body.size());
            CHECK(output.good());
        }
    }

    ~TemporaryFile()
    {
        if (!path.empty()) {
            CHECK(unlink(path.c_str()) == 0);
        }
    }

    const char *name() const
    {
        return path.c_str();
    }

private:
    std::string path;
};

std::string md5_text(const std::vector<uint8_t> &body)
{
    MD5Context context;
    uint8_t digest[16];
    char encoded[33] {};
    MD5Init(&context);
    MD5Update(&context, body.data(), body.size());
    MD5Final(digest, &context);
    for (uint8_t index = 0; index < sizeof(digest); index++) {
        snprintf(&encoded[index * 2U], 3, "%02x", digest[index]);
    }
    return encoded;
}

std::string header_for(const std::vector<uint8_t> &body)
{
    return "MD5: " + md5_text(body) + "\n--\n";
}

template <typename Descriptor>
std::vector<uint8_t> make_body(uint32_t body_size, uint32_t descriptor_offset,
                               uint16_t board_id = BOARD_ID)
{
    std::vector<uint8_t> body(body_size, 0x5A);
    CHECK(descriptor_offset + sizeof(Descriptor) <= body.size());
    Descriptor descriptor {};
    const uint8_t unsigned_signature[] = AP_APP_DESCRIPTOR_SIGNATURE_UNSIGNED;
    const uint8_t signed_signature[] = AP_APP_DESCRIPTOR_SIGNATURE_SIGNED;
    const uint8_t *signature = sizeof(Descriptor) == sizeof(app_descriptor_signed) ?
                               signed_signature : unsigned_signature;
    memcpy(descriptor.sig, signature, sizeof(descriptor.sig));
    descriptor.image_size = body.size();
    descriptor.git_hash = 0x12345678U;
    descriptor.version_major = 4;
    descriptor.version_minor = 6;
    descriptor.board_id = board_id;
    memcpy(&body[descriptor_offset], &descriptor, sizeof(descriptor));

    descriptor.image_crc1 = crc32_small(
        0, body.data(), descriptor_offset + offsetof(Descriptor, image_crc1));
    const uint32_t crc2_offset = descriptor_offset + offsetof(Descriptor, version_major);
    descriptor.image_crc2 = crc32_small(
        0, &body[crc2_offset], body.size() - crc2_offset);
    memcpy(&body[descriptor_offset], &descriptor, sizeof(descriptor));
    return body;
}

bool validates(const std::vector<uint8_t> &body, uint32_t maximum_size = IMAGE_SIZE_LIMIT,
               uint16_t board_id = BOARD_ID)
{
    TemporaryFile file(header_for(body), body);
    return abin_validate(file.name(), maximum_size, board_id);
}

void test_valid_descriptors()
{
    const auto unsigned_body = make_body<app_descriptor_unsigned>(5000, 4093);
    CHECK(validates(unsigned_body));

    const auto signed_body = make_body<app_descriptor_signed>(6000, 4093);
    CHECK(validates(signed_body));
}

void test_header_rejections()
{
    const auto body = make_body<app_descriptor_unsigned>(2048, 256);
    const std::string md5 = md5_text(body);
    const std::vector<std::string> invalid_headers {
        "--\n",
        "MD5: " + md5 + "\nMD5: " + md5 + "\n--\n",
        "MD5: " + md5.substr(1) + "x\n--\n",
        "MD5: " + md5 + "\n",
        std::string(96, 'A') + "\nMD5: " + md5 + "\n--\n",
    };
    for (const auto &header : invalid_headers) {
        TemporaryFile file(header, body);
        CHECK(!abin_validate(file.name(), IMAGE_SIZE_LIMIT, BOARD_ID));
    }

    TemporaryFile empty("MD5: d41d8cd98f00b204e9800998ecf8427e\n--\n", {});
    CHECK(!abin_validate(empty.name(), IMAGE_SIZE_LIMIT, BOARD_ID));
}

void test_integrity_rejections()
{
    auto body = make_body<app_descriptor_unsigned>(5000, 4093);
    CHECK(!validates(body, 4992));
    CHECK(!validates(body, IMAGE_SIZE_LIMIT, BOARD_ID + 1U));

    TemporaryFile bad_md5("MD5: 00000000000000000000000000000000\n--\n", body);
    CHECK(!abin_validate(bad_md5.name(), IMAGE_SIZE_LIMIT, BOARD_ID));

    body[0] ^= 1U;
    TemporaryFile stale_md5(header_for(make_body<app_descriptor_unsigned>(5000, 4093)), body);
    CHECK(!abin_validate(stale_md5.name(), IMAGE_SIZE_LIMIT, BOARD_ID));
}

void test_descriptor_rejections()
{
    const uint32_t offset = 512;
    auto body = make_body<app_descriptor_unsigned>(4096, offset);

    auto bad_size = body;
    reinterpret_cast<app_descriptor_unsigned *>(&bad_size[offset])->image_size--;
    TemporaryFile bad_size_file(header_for(bad_size), bad_size);
    CHECK(!abin_validate(bad_size_file.name(), IMAGE_SIZE_LIMIT, BOARD_ID));

    auto bad_crc1 = body;
    reinterpret_cast<app_descriptor_unsigned *>(&bad_crc1[offset])->image_crc1 ^= 1U;
    TemporaryFile bad_crc1_file(header_for(bad_crc1), bad_crc1);
    CHECK(!abin_validate(bad_crc1_file.name(), IMAGE_SIZE_LIMIT, BOARD_ID));

    auto bad_crc2 = body;
    reinterpret_cast<app_descriptor_unsigned *>(&bad_crc2[offset])->image_crc2 ^= 1U;
    TemporaryFile bad_crc2_file(header_for(bad_crc2), bad_crc2);
    CHECK(!abin_validate(bad_crc2_file.name(), IMAGE_SIZE_LIMIT, BOARD_ID));

    auto duplicate = make_body<app_descriptor_unsigned>(4096, 2048);
    const uint8_t signature[] = AP_APP_DESCRIPTOR_SIGNATURE_UNSIGNED;
    memcpy(&duplicate[512], signature, sizeof(signature));
    auto *last_descriptor =
        reinterpret_cast<app_descriptor_unsigned *>(&duplicate[2048]);
    last_descriptor->image_crc1 = crc32_small(
        0, duplicate.data(), 2048 + offsetof(app_descriptor_unsigned, image_crc1));
    TemporaryFile duplicate_file(header_for(duplicate), duplicate);
    CHECK(!abin_validate(duplicate_file.name(), IMAGE_SIZE_LIMIT, BOARD_ID));

    std::vector<uint8_t> absent(4096, 0x5A);
    TemporaryFile absent_file(header_for(absent), absent);
    CHECK(!abin_validate(absent_file.name(), IMAGE_SIZE_LIMIT, BOARD_ID));
}

class RecordingSink : public ABinBodySink {
public:
    bool write(const uint8_t *bytes, uint32_t size) override
    {
        if (!accept_writes) {
            return false;
        }
        received.insert(received.end(), bytes, bytes + size);
        return true;
    }

    bool finish() override
    {
        finished = true;
        return accept_finish;
    }

    std::vector<uint8_t> received;
    bool accept_writes = true;
    bool accept_finish = true;
    bool finished = false;
};

void test_streaming()
{
    const auto body = make_body<app_descriptor_unsigned>(9000, 4093);
    TemporaryFile file(header_for(body), body);

    RecordingSink sink;
    CHECK(abin_stream_body(file.name(), sink));
    CHECK(sink.finished);
    CHECK(sink.received == body);

    RecordingSink rejected_write;
    rejected_write.accept_writes = false;
    CHECK(!abin_stream_body(file.name(), rejected_write));
    CHECK(!rejected_write.finished);

    RecordingSink rejected_finish;
    rejected_finish.accept_finish = false;
    CHECK(!abin_stream_body(file.name(), rejected_finish));
    CHECK(rejected_finish.finished);
}

} // namespace

int main()
{
    test_valid_descriptors();
    test_header_rejections();
    test_integrity_rejections();
    test_descriptor_rejections();
    test_streaming();
    if (failures != 0) {
        fprintf(stderr, "%d ABin checks failed\n", failures);
        return 1;
    }
    printf("ABin validation tests passed\n");
    return 0;
}
