#pragma once

#include <AP_Beacon/AP_Beacon.h>

#if AP_BEACON_ENABLED

#include <AP_Logger/LogStructure.h>

class AP_DAL_Beacon {
public:

    // Beacon-like methods:
    uint8_t count() const {
        return _RBCH.count;
    }

    bool get_origin(Location &loc) const {
        loc.zero();
        loc.lat = _RBCH.origin_lat;
        loc.lng = _RBCH.origin_lng;
        loc.alt = _RBCH.origin_alt;
        return _RBCH.get_origin_returncode;
    }

    // return beacon enabled
    bool enabled(void) const {
        return _RBCH.enabled;
    }

    // return beacon health
    bool beacon_healthy(uint8_t i) const {
        return _RBCI[i].healthy;
    }

    // return last update time from beacon in milliseconds
    uint32_t beacon_last_update_ms(uint8_t i) const {
        return _RBCI[i].last_update_ms;
    }

    // return distance to beacon in meters
    float beacon_distance(uint8_t i) const {
        return _RBCI[i].distance;
    }

    // return NED position of beacon in meters relative to the beacon systems origin
    const Vector3f &beacon_position(uint8_t i) const {
        return _RBCI[i].position;
    }

    uint8_t tdoa_count() const {
        return _tdoa_count;
    }

    bool get_tdoa_data(uint8_t i, AP_Beacon::TDoAState &state) const {
        if (i >= _tdoa_count) {
            return false;
        }

        const log_RBTD &RBTD = _RBTD[i];
        state.anchor_id_a = RBTD.anchor_id_a;
        state.anchor_id_b = RBTD.anchor_id_b;
        state.distance_diff = RBTD.distance_diff;
        state.distance_diff_err = RBTD.distance_diff_err;
        state.healthy = RBTD.healthy;
        state.update_ms = RBTD.last_update_ms;
        return true;
    }

    // return vehicle position in NED from position estimate system's origin in meters
    bool get_vehicle_position_ned(Vector3f& pos, float& accuracy_estimate) const {
        pos = _RBCH.vehicle_position_ned;
        accuracy_estimate = _RBCH.accuracy_estimate;
        return _RBCH.get_vehicle_position_ned_returncode;
    }

    // AP_DAL methods:
    AP_DAL_Beacon();

    AP_DAL_Beacon *beacon() {
        return this;
    }

    void start_frame();

    void handle_message(const log_RBCH &msg) {
        _RBCH = msg;
    }
    void handle_message(const log_RBCI &msg) {
        if (msg.instance < ARRAY_SIZE(_RBCI)) {
            _RBCI[msg.instance] = msg;
        }
    }
    void handle_message(const log_RBTD &msg) {
        if (msg.instance < ARRAY_SIZE(_RBTD)) {
            _RBTD[msg.instance] = msg;
            if (msg.instance + 1 > _tdoa_count) {
                _tdoa_count = msg.instance + 1;
            }
        }
    }

private:

    struct log_RBCH _RBCH;
    struct log_RBCI _RBCI[AP_BEACON_MAX_BEACONS];
    struct log_RBTD _RBTD[AP_BEACON_MAX_TDOA_MEASUREMENTS];
    uint8_t _tdoa_count = 0;
};

#endif  // AP_BEACON_ENABLED
