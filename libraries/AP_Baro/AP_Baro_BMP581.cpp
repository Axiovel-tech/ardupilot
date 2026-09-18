/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "AP_Baro_BMP581.h"

#if AP_BARO_BMP581_ENABLED

#include <AP_Math/AP_Math.h>

extern const AP_HAL::HAL &hal;

#define BMP581_ID                     0x50

#define BMP581_REG_CHIP_ID            0x01
#define BMP581_REG_REV_ID             0x02
#define BMP581_REG_CHIP_STATUS        0x11
#define BMP581_REG_DRIVE_CONFIG       0x13
#define BMP581_REG_INT_CONFIG         0x14
#define BMP581_REG_INT_SOURCE         0x15
#define BMP581_REG_FIFO_CONFIG        0x16
#define BMP581_REG_FIFO_COUNT         0x17
#define BMP581_REG_FIFO_SEL           0x18
#define BMP581_REG_TEMP_DATA_XLSB     0x1D
#define BMP581_REG_TEMP_DATA_LSB      0x1E
#define BMP581_REG_TEMP_DATA_MSB      0x1F
#define BMP581_REG_PRESS_DATA_XLSB    0x20
#define BMP581_REG_PRESS_DATA_LSB     0x21
#define BMP581_REG_PRESS_DATA_MSB     0x22
#define BMP581_REG_INT_STATUS         0x27
#define BMP581_REG_STATUS             0x28
#define BMP581_REG_FIFO_DATA          0x29
#define BMP581_REG_NVM_ADDR           0x2B
#define BMP581_REG_NVM_DATA_LSB       0x2C
#define BMP581_REG_NVM_DATA_MSB       0x2D
#define BMP581_REG_DSP_CONFIG         0x30
#define BMP581_REG_DSP_IIR            0x31
#define BMP581_REG_OOR_THR_P_LSB      0x32
#define BMP581_REG_OOR_THR_P_MSB      0x33
#define BMP581_REG_OOR_RANGE          0x34
#define BMP581_REG_OOR_CONFIG         0x35
#define BMP581_REG_OSR_CONFIG         0x36
#define BMP581_REG_ODR_CONFIG         0x37
#define BMP581_REG_OSR_EFF            0x38
#define BMP581_REG_CMD                0x7E

#define BMP581_CMD_SOFT_RESET         0xB6

// STATUS register (0x28) bits
#define BMP581_STATUS_NVM_RDY         (1U << 1)
#define BMP581_STATUS_NVM_ERR         (1U << 2)

// INT_STATUS register (0x27): set on power-on or soft reset, cleared on read
#define BMP581_INT_STATUS_POR         (1U << 4)

// datasheet soft-reset duration is 2ms; use a margin
#define BMP581_RESET_DELAY_MS         5

// Keep both sensor output and host polling at 50Hz. 64x pressure OSR only
// supports 20Hz: at 50Hz the sensor silently falls back to 2x/2x instead.
// 16x pressure / 4x temperature is valid up to 80Hz (datasheet table 7).
#define BMP581_OSR_CONFIG             ((1U << 6) | (4U << 3) | 2U)
#define BMP581_ODR_CONFIG             ((0x0FU << 2) | 1U)
#define BMP581_OSR_EFF_MASK           0xBFU
#define BMP581_OSR_EFF_EXPECTED       ((1U << 7) | (4U << 3) | 2U)

AP_Baro_BMP581::AP_Baro_BMP581(AP_Baro &baro, AP_HAL::Device &dev)
    : AP_Baro_Backend(baro)
    , _dev(&dev)
{
}

AP_Baro_Backend *AP_Baro_BMP581::probe(AP_Baro &baro, AP_HAL::Device &dev)
{
    AP_Baro_BMP581 *sensor = NEW_NOTHROW AP_Baro_BMP581(baro, dev);
    if (!sensor || !sensor->init()) {
        delete sensor;
        return nullptr;
    }
    return sensor;
}

bool AP_Baro_BMP581::init()
{
    if (!_dev) {
        return false;
    }

    WITH_SEMAPHORE(_dev->get_semaphore());

    _dev->set_speed(AP_HAL::Device::SPEED_HIGH);

    uint8_t whoami;

    // setup to allow reads on SPI
    if (_dev->bus_type() == AP_HAL::Device::BUS_TYPE_SPI) {
        _dev->set_read_flag(0x80);

        if (!_dev->read_registers(BMP581_REG_CHIP_ID, &whoami, 1)) {
            return false;
        }
    }

    if (!_dev->read_registers(BMP581_REG_CHIP_ID, &whoami, 1)) {
        return false;
    }

    switch (whoami) {
    case BMP581_ID:
        _dev->set_device_type(DEVTYPE_BARO_BMP581);
        break;
    default:
        return false;
    }

    // On a warm reboot the MCU restarts but the BMP581 can stay powered, and
    // init() then rejects a working sensor: STATUS reads 0x01 (core_rdy=1,
    // nvm_rdy=0, nvm_err=0) and the INT_STATUS POR bit is 0, though pressure
    // output is still valid. Reset establishes a known startup state so the
    // STATUS/INT_STATUS checks below apply to a freshly reset sensor rather
    // than one left measuring by the previous boot. The I2C reset write may not be
    // ACKed, so its result is intentionally ignored; the reads below confirm
    // the state.
    _dev->write_register(BMP581_REG_CMD, BMP581_CMD_SOFT_RESET);
    hal.scheduler->delay(BMP581_RESET_DELAY_MS);

    // A soft reset drops SPI back to the chip's default interface. The first
    // SPI read afterwards is a dummy whose value is discarded to re-lock SPI;
    // then confirm the ID still reads back before continuing.
    if (_dev->bus_type() == AP_HAL::Device::BUS_TYPE_SPI) {
        uint8_t dummy;
        _dev->read_registers(BMP581_REG_CHIP_ID, &dummy, 1);

        if (!_dev->read_registers(BMP581_REG_CHIP_ID, &whoami, 1) || whoami != BMP581_ID) {
            return false;
        }
    }

    uint8_t status;
    if (!_dev->read_registers(BMP581_REG_STATUS, &status, 1)) {
        return false;
    }

    if ((status & BMP581_STATUS_NVM_RDY) == 0 || (status & BMP581_STATUS_NVM_ERR)) {
        return false;
    }

    uint8_t int_status;
    if (!_dev->read_registers(BMP581_REG_INT_STATUS, &int_status, 1)) {
        return false;
    }

    if ((int_status & BMP581_INT_STATUS_POR) == 0) {
        return false;
    }

    if (!_dev->setup_checked_registers(2)) {
        return false;
    }

    // Standby is transient, not a checked runtime setting. Allow the maximum
    // 2.5ms transition time before requesting another mode (datasheet 4.3.7).
    if (!_dev->write_register(BMP581_REG_ODR_CONFIG, 0)) {
        return false;
    }
    hal.scheduler->delay(3);

    if (!_dev->write_register(BMP581_REG_OSR_CONFIG, BMP581_OSR_CONFIG, true) ||
        !_dev->write_register(BMP581_REG_ODR_CONFIG, BMP581_ODR_CONFIG, true)) {
        return false;
    }

    // Allow startup and a measurement period before checking the effective
    // configuration. ACKed writes alone do not prove the settings took effect.
    hal.scheduler->delay(25);
    uint8_t config[2];
    uint8_t osr_eff;
    if (!_dev->read_registers(BMP581_REG_OSR_CONFIG, config, sizeof(config)) ||
        config[0] != BMP581_OSR_CONFIG || config[1] != BMP581_ODR_CONFIG ||
        !_dev->read_registers(BMP581_REG_OSR_EFF, &osr_eff, 1) ||
        (osr_eff & BMP581_OSR_EFF_MASK) != BMP581_OSR_EFF_EXPECTED) {
        return false;
    }

    instance = _frontend.register_sensor();

    set_bus_id(instance, _dev->get_bus_id());

    // request 50Hz update
    _dev->register_periodic_callback(20 * AP_USEC_PER_MSEC, FUNCTOR_BIND_MEMBER(&AP_Baro_BMP581::timer, void));

    return true;
}

// acumulate a new sensor reading
void AP_Baro_BMP581::timer(void)
{
    // Keep configuration recovery running even when sample reads fail or
    // disagree; otherwise those early returns can starve register checking.
    if (!_dev->check_next_register()) {
        return;
    }

    uint8_t buf[6];
    uint8_t buf2[6];

    // read twice, make sure results are consistent; corruption has
    // been seen from data on this sensor
    if (!_dev->read_registers(BMP581_REG_TEMP_DATA_XLSB, buf, sizeof(buf))) {
        return;
    }
    if (!_dev->read_registers(BMP581_REG_TEMP_DATA_XLSB, buf2, sizeof(buf2))) {
        return;
    }
    if (memcmp(buf, buf2, ARRAY_SIZE(buf)) != 0) {
        // we didn't get the same data twice.  Reject.  See if we can
        // get a third measurement to match the second:
        if (!_dev->read_registers(BMP581_REG_TEMP_DATA_XLSB, buf, sizeof(buf))) {
            return;
        }
        if (memcmp(buf, buf2, ARRAY_SIZE(buf)) != 0) {
            return;
        }
    }

    WITH_SEMAPHORE(_sem);

    if (buf[0] != 0x7f || buf[1] != 0x7f || buf[2] != 0x7f) {
        // we have temperature data
        temperature = (float)((int32_t)(((uint32_t)buf[2] << 24) | ((uint32_t)buf[1] << 16) | ((uint32_t)buf[0] << 8)) >> 8) * (1.0f / 65536.0f);
    }

    if (buf[3] != 0x7f || buf[4] != 0x7f || buf[5] != 0x7f) {
        // we have pressure data
        pressure_sum += (float)(((uint32_t)buf[5] << 16) | ((uint32_t)buf[4] << 8) | (uint32_t)buf[3]) * (1.0f / 64.0f);
        pressure_count++;
    }
}

// transfer data to the frontend
void AP_Baro_BMP581::update(void)
{
    WITH_SEMAPHORE(_sem);

    if (pressure_count == 0) {
        return;
    }

    _copy_to_frontend(instance,
                      pressure_sum/pressure_count,
                      temperature);

    pressure_sum = 0;
    pressure_count = 0;
}

#endif  // AP_BARO_BMP581_ENABLED
