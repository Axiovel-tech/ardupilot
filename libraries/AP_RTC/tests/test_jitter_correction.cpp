#include <AP_gtest.h>

#include <AP_HAL/AP_HAL.h>
#include <AP_RTC/JitterCorrection.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

static constexpr uint64_t transport_us = 3000;
static constexpr uint64_t period_us = 20000;

// a remote stream with a constant transport lag; returns the last estimate's lag
static int64_t stream(JitterCorrection &jc, uint64_t &remote_us, uint64_t &local_us, uint16_t samples)
{
    int64_t lag_us = 0;
    for (uint16_t i = 0; i < samples; i++) {
        remote_us += period_us;
        local_us += period_us;
        lag_us = int64_t(local_us) - int64_t(jc.correct_offboard_timestamp_usec(remote_us, local_us));
    }
    return lag_us;
}

TEST(JitterCorrection, constant_lag_stream_maps_to_arrival)
{
    JitterCorrection jc{500, 100};
    uint64_t remote_us = 600000000;
    uint64_t local_us = remote_us + 1000000 + transport_us;
    EXPECT_EQ(stream(jc, remote_us, local_us, 10), int64_t(0));
}

TEST(JitterCorrection, remote_restart_takes_effect_at_once)
{
    JitterCorrection jc{500, 100};
    uint64_t remote_us = 600000000;
    uint64_t local_us = remote_us + 1000000 + transport_us;
    stream(jc, remote_us, local_us, 150);

    // the remote reboots: 8 s of silence, then its clock restarts near zero
    remote_us = 2000000;
    local_us += 8000000;
    for (uint16_t i = 0; i < 300; i++) {
        remote_us += period_us;
        local_us += period_us;
        const uint64_t estimate_us = jc.correct_offboard_timestamp_usec(remote_us, local_us);
        EXPECT_EQ(local_us - estimate_us, uint64_t(0));
    }
}

TEST(JitterCorrection, reordering_within_max_lag_keeps_offset)
{
    JitterCorrection jc{500, 100};
    uint64_t remote_us = 600000000;
    uint64_t local_us = remote_us + 1000000 + transport_us;
    stream(jc, remote_us, local_us, 50);
    const int64_t offset_us = jc.get_link_offset_usec();

    // an older sample (another message type of the same stream) arrives late
    jc.correct_offboard_timestamp_usec(remote_us - 100000, local_us + 1000);
    EXPECT_EQ(jc.get_link_offset_usec(), offset_us);
}

AP_GTEST_MAIN()
