/*
 * Exercise the production BMP581 backend through its public probe/update API
 * and the periodic callback registered with a fake register-level I2C device.
 * No driver implementation is copied into this test.
 *
 * ./waf configure --board sitl --debug --no-submodule-update
 * ./waf --target tests/test_baro_bmp581
 * build/sitl/tests/test_baro_bmp581
 */
#include <AP_gtest.h>
#include <AP_Baro/AP_Baro_BMP581.h>
#include <AP_HAL/utility/OwnPtr.h>

#include <array>
#include <vector>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

#if AP_BARO_BMP581_ENABLED

namespace {

class BMP581Device : public AP_HAL::Device {
public:
    explicit BMP581Device(BusType bus_type = BUS_TYPE_I2C) : Device(bus_type)
    {
        // Cold-power startup. Individual tests also model MCU-only reboots.
        registers[0x01] = 0x50;
        registers[0x27] = 0x10;
        registers[0x28] = 0x03;
        registers[0x37] = 0x70; // Reset ODR_CONFIG: deep standby.
    }

    bool set_speed(Speed speed) override { return true; }
    AP_HAL::Semaphore *get_semaphore() override { return &semaphore; }
    PeriodicHandle register_periodic_callback(uint32_t period_usec, PeriodicCb cb) override
    {
        period_us = period_usec;
        callback = cb;
        return this;
    }
    bool adjust_periodic_callback(PeriodicHandle handle, uint32_t period_usec) override
    {
        return false;
    }

    bool transfer(const uint8_t *send, uint32_t send_len,
                  uint8_t *recv, uint32_t recv_len) override
    {
        if (send_len == 2 && recv_len == 0) {
            const uint8_t reg = send[0];
            const uint8_t value = send[1];
            writes[reg]++;
            if (reg == 0x7e && value == 0xb6) {
                reset_time_us = AP_HAL::micros64();
                reset_count++;
                if (reset_takes_effect) {
                    registers[0x01] = reset_chip_id;
                    registers[0x27] = reset_int_status;
                    registers[0x28] = reset_status;
                    registers[0x36] = 0;
                    registers[0x37] = 0x70;
                }
                return reset_ack;
            }
            if (reg == fail_write_reg && value == fail_write_value) {
                return false;
            }
            if (reg == 0x37 && value == 0) {
                standby_time_us = AP_HAL::micros64();
            }
            if (reg == 0x36) {
                osr_time_us = AP_HAL::micros64();
            }
            if (reg == 0x37 && (value & 3) == 1) {
                normal_time_us = AP_HAL::micros64();
            }
            if (reg != ignore_write_reg) {
                registers[reg] = value;
            }
            return true;
        }
        if (send_len != 1 || recv_len == 0) {
            return false;
        }
        // This models register access/control flow, not SPI electrical framing.
        const uint8_t reg = send[0] & (bus_type() == BUS_TYPE_SPI ? 0x7f : 0xff);
        if (reg + recv_len > sizeof(registers)) {
            return false;
        }
        reads[reg]++;
        if (reg == fail_read_reg) {
            return false;
        }
        if (reset_count != 0 && first_post_reset_read_us == 0) {
            first_post_reset_read_us = AP_HAL::micros64();
        }
        if (reg == 0x1d && recv_len == 6) {
            const size_t index = data_read_count++;
            if (index == fail_data_read || frames.empty()) {
                return false;
            }
            const size_t frame_index = MIN(index, frames.size() - 1);
            memcpy(recv, frames[frame_index].data(), recv_len);
            return true;
        }
        if (reg == 0x38 && recv_len == 1) {
            effective_read_time_us = AP_HAL::micros64();
            recv[0] = effective_osr;
            return true;
        }
        memcpy(recv, &registers[reg], recv_len);
        if (reg == 0x27) {
            registers[0x27] = 0;  // INT_STATUS is clear-on-read.
        }
        return true;
    }

    void tick() { callback(); }

    // BMP581 pressure is unsigned Q18.6; temperature is signed Q8.16.
    static std::array<uint8_t, 6> sample(uint32_t pressure_pa, int32_t temperature_c = 25)
    {
        const uint32_t p = pressure_pa * 64;
        const uint32_t t = uint32_t(temperature_c * 65536);
        return {{uint8_t(t), uint8_t(t >> 8), uint8_t(t >> 16),
                 uint8_t(p), uint8_t(p >> 8), uint8_t(p >> 16)}};
    }

    uint8_t registers[128] {};
    uint32_t reads[128] {};
    uint32_t writes[128] {};
    bool reset_ack = true;
    bool reset_takes_effect = true;
    uint8_t reset_chip_id = 0x50;
    uint8_t reset_status = 0x03;
    uint8_t reset_int_status = 0x10;
    uint8_t effective_osr = 0xa2; // valid 16x pressure / 4x temperature
    uint8_t fail_write_reg = 0xff;
    uint8_t fail_write_value = 0xff;
    uint8_t ignore_write_reg = 0xff;
    uint8_t fail_read_reg = 0xff;
    size_t fail_data_read = SIZE_MAX;
    size_t data_read_count = 0;
    uint32_t reset_count = 0;
    uint32_t period_us = 0;
    uint64_t reset_time_us = 0;
    uint64_t first_post_reset_read_us = 0;
    uint64_t standby_time_us = 0;
    uint64_t osr_time_us = 0;
    uint64_t normal_time_us = 0;
    uint64_t effective_read_time_us = 0;
    std::vector<std::array<uint8_t, 6>> frames;
    PeriodicCb callback;
    HAL_Semaphore semaphore;
};

class BMP581Test : public ::testing::Test {
protected:
    // ArduPilot's heap allocation zero-initializes backend/frontend storage.
    AP_HAL::OwnPtr<AP_Baro> baro {NEW_NOTHROW AP_Baro};
    AP_HAL::OwnPtr<BMP581Device> device {NEW_NOTHROW BMP581Device};
    AP_HAL::OwnPtr<AP_Baro_Backend> backend;

    void probe() { backend = AP_Baro_BMP581::probe(*baro, *device); }
};

TEST_F(BMP581Test, WarmBootResetsSensorAndUsesValid50HzConfiguration)
{
    // Sensor stayed powered and its previous driver consumed the POR flag.
    device->registers[0x27] = 0;
    device->registers[0x28] = 0x01;
    device->registers[0x37] = 0x3d;
    probe();
    ASSERT_TRUE(bool(backend));
    EXPECT_EQ(device->reset_count, 1U);
    EXPECT_GE(device->first_post_reset_read_us - device->reset_time_us, 2000U);
    EXPECT_GE(device->osr_time_us - device->standby_time_us, 2500U);
    EXPECT_GE(device->effective_read_time_us - device->normal_time_us, 20000U);
    EXPECT_EQ(device->registers[0x36], 0x62);
    EXPECT_EQ(device->registers[0x37], 0x3d);
    EXPECT_EQ(device->period_us, 20000U);
    EXPECT_TRUE(bool(device->callback));
}

TEST_F(BMP581Test, ResetMayCompleteWithoutI2CAcknowledgement)
{
    device->registers[0x27] = 0;
    device->registers[0x28] = 0x01;
    device->reset_ack = false;
    probe();
    ASSERT_TRUE(bool(backend));
    EXPECT_EQ(device->reset_count, 1U);
}

TEST_F(BMP581Test, FailedResetDoesNotRegisterSensor)
{
    device->registers[0x27] = 0;
    device->registers[0x28] = 0x01;
    device->reset_takes_effect = false;
    device->reset_ack = false;
    probe();
    EXPECT_FALSE(bool(backend));
    EXPECT_FALSE(bool(device->callback));
}

TEST_F(BMP581Test, WrongChipDoesNotReceiveReset)
{
    device->registers[0x01] = 0x51;
    probe();
    EXPECT_FALSE(bool(backend));
    EXPECT_EQ(device->reset_count, 0U);
}

TEST_F(BMP581Test, SPIControlFlowIncludesDummyAndIdentityReadsBeforeAndAfterReset)
{
    device = NEW_NOTHROW BMP581Device(AP_HAL::Device::BUS_TYPE_SPI);
    probe();
    ASSERT_TRUE(bool(backend));
    EXPECT_EQ(device->reads[0x01], 4U);
    EXPECT_EQ(device->reset_count, 1U);
    EXPECT_GE(device->first_post_reset_read_us - device->reset_time_us, 2000U);
}

TEST_F(BMP581Test, SPIControlFlowRechecksChipIdentityAfterReset)
{
    device = NEW_NOTHROW BMP581Device(AP_HAL::Device::BUS_TYPE_SPI);
    device->reset_chip_id = 0;
    probe();
    EXPECT_FALSE(bool(backend));
    EXPECT_FALSE(bool(device->callback));
}

TEST_F(BMP581Test, NVMNotReadyFailsInitialization)
{
    device->reset_status = 0x01;
    probe();
    EXPECT_FALSE(bool(backend));
}

TEST_F(BMP581Test, NVMErrorFailsInitialization)
{
    device->reset_status = 0x07;
    probe();
    EXPECT_FALSE(bool(backend));
}

TEST_F(BMP581Test, MissingResetCompleteFlagFailsInitialization)
{
    device->reset_int_status = 0;
    probe();
    EXPECT_FALSE(bool(backend));
}

TEST_F(BMP581Test, StandbyWriteFailureFailsInitialization)
{
    device->fail_write_reg = 0x37;
    device->fail_write_value = 0;
    probe();
    EXPECT_FALSE(bool(backend));
}

TEST_F(BMP581Test, OversamplingWriteFailureFailsInitialization)
{
    device->fail_write_reg = 0x36;
    device->fail_write_value = 0x62;
    probe();
    EXPECT_FALSE(bool(backend));
}

TEST_F(BMP581Test, NormalModeWriteFailureFailsInitialization)
{
    device->fail_write_reg = 0x37;
    device->fail_write_value = 0x3d;
    probe();
    EXPECT_FALSE(bool(backend));
}

TEST_F(BMP581Test, AcknowledgedButIgnoredConfigurationFailsInitialization)
{
    device->ignore_write_reg = 0x36;
    probe();
    EXPECT_FALSE(bool(backend));
}

TEST_F(BMP581Test, InvalidEffectiveOversamplingFailsInitialization)
{
    device->effective_osr = 0x09; // invalid ODR/OSR: fallback 2x pressure / 2x temperature
    probe();
    EXPECT_FALSE(bool(backend));
}

TEST_F(BMP581Test, ValidButUnexpectedEffectiveOversamplingFailsInitialization)
{
    device->effective_osr = 0x89; // valid bit alone is insufficient
    probe();
    EXPECT_FALSE(bool(backend));
}

TEST_F(BMP581Test, ReservedEffectiveOversamplingBitIsIgnored)
{
    device->effective_osr = 0xe2;
    probe();
    EXPECT_TRUE(bool(backend));
}

TEST_F(BMP581Test, EffectiveOversamplingReadFailureFailsInitialization)
{
    device->fail_read_reg = 0x38;
    probe();
    EXPECT_FALSE(bool(backend));
}

TEST_F(BMP581Test, StableReadPublishesPressureAndSignedTemperature)
{
    probe();
    ASSERT_TRUE(bool(backend));
    device->frames = {BMP581Device::sample(100000, -10)};
    device->tick();
    backend->update();
    EXPECT_EQ(device->data_read_count, 2U);
    EXPECT_FLOAT_EQ(baro->get_pressure(0), 100000);
    EXPECT_FLOAT_EQ(baro->get_temperature(0), -10);
}

TEST_F(BMP581Test, ConversionBetweenReadsAcceptsMatchingSecondAndThirdFrames)
{
    probe();
    ASSERT_TRUE(bool(backend));
    device->frames = {BMP581Device::sample(100000), BMP581Device::sample(100010),
                      BMP581Device::sample(100010)};
    device->tick();
    backend->update();
    EXPECT_EQ(device->data_read_count, 3U);
    EXPECT_FLOAT_EQ(baro->get_pressure(0), 100010);
}

TEST_F(BMP581Test, ThreeDifferentFramesAreRejected)
{
    probe();
    ASSERT_TRUE(bool(backend));
    device->frames = {BMP581Device::sample(100000), BMP581Device::sample(100010),
                      BMP581Device::sample(100020)};
    device->tick();
    backend->update();
    EXPECT_FLOAT_EQ(baro->get_pressure(0), 0);
}

TEST_F(BMP581Test, FailedThirdReadIsRejected)
{
    probe();
    ASSERT_TRUE(bool(backend));
    device->frames = {BMP581Device::sample(100000), BMP581Device::sample(100010)};
    device->fail_data_read = 2;
    device->tick();
    backend->update();
    EXPECT_FLOAT_EQ(baro->get_pressure(0), 0);
}

TEST_F(BMP581Test, FailedFirstReadIsRejected)
{
    probe();
    ASSERT_TRUE(bool(backend));
    device->frames = {BMP581Device::sample(100000)};
    device->fail_data_read = 0;
    device->tick();
    backend->update();
    EXPECT_EQ(device->data_read_count, 1U);
    EXPECT_FLOAT_EQ(baro->get_pressure(0), 0);
}

TEST_F(BMP581Test, FailedSecondReadIsRejected)
{
    probe();
    ASSERT_TRUE(bool(backend));
    device->frames = {BMP581Device::sample(100000)};
    device->fail_data_read = 1;
    device->tick();
    backend->update();
    EXPECT_EQ(device->data_read_count, 2U);
    EXPECT_FLOAT_EQ(baro->get_pressure(0), 0);
}

TEST_F(BMP581Test, InvalidPressureSentinelIsNotPublished)
{
    probe();
    ASSERT_TRUE(bool(backend));
    device->frames = {{{0, 0, 25, 0x7f, 0x7f, 0x7f}}};
    device->tick();
    backend->update();
    EXPECT_FLOAT_EQ(baro->get_pressure(0), 0);
}

TEST_F(BMP581Test, RegisterChecksContinueWhenAllSampleReadsFail)
{
    probe();
    ASSERT_TRUE(bool(backend));
    // Simulate normal mode lost after initialization, with no usable samples.
    device->registers[0x37] = 0;
    for (uint8_t i = 0; i < 30; i++) {
        device->tick();
    }
    EXPECT_EQ(device->registers[0x37], 0x3d);
    EXPECT_GT(device->reads[0x37], 0U);
}

} // namespace

#endif // AP_BARO_BMP581_ENABLED

AP_GTEST_MAIN()
