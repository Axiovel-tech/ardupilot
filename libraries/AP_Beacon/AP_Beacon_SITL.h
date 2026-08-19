#pragma once

#include "AP_Beacon_Backend.h"

#if AP_BEACON_SITL_ENABLED

#include <SITL/SITL.h>

class AP_Beacon_SITL : public AP_Beacon_Backend
{

public:
    // constructor
    AP_Beacon_SITL(AP_Beacon &frontend);

    // return true if sensor is basically healthy (we are receiving data)
    bool healthy() override;

    // update
    void update() override;

private:
    SITL::SIM *sitl;
    uint8_t next_beacon;
    uint8_t next_tdoa_pair;
    uint32_t last_update_ms;
    uint32_t tdoa_rng_state;
    uint32_t rng_rng_state;
    int32_t last_tdoa_seed;
    int32_t last_rng_seed;
    bool tdoa_seed_initialized;
    bool rng_seed_initialized;
    bool startup_position_complete;

    void reset_tdoa_rng_if_needed();
    float tdoa_rand_float();
    float tdoa_rand_normal();
    void reset_rng_rng_if_needed();
    float rng_rand_float();
    float rng_rand_normal();
};

#endif // AP_BEACON_SITL_ENABLED
