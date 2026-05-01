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

#include "AP_Beacon_SITL.h"

#if AP_BEACON_SITL_ENABLED

#include <AP_HAL/AP_HAL.h>

#include <stdio.h>

extern const AP_HAL::HAL& hal;

#define NUM_BEACONS 4
/*
 *  Define a rectangular pattern of beacons with the pattern centroid located at the beacon origin as defined by the following params:
 *
 * BCN_ALT - Height above the WGS-84 geoid (m)
 * BCN_LATITUDE - WGS-84 latitude (deg)
 * BCN_LONGITUDE - WGS-84 longitude (deg)
 *
 * The spacing between beacons in the North/South and East/West directions is defined by the following parameters:
 */
#define BEACON_SPACING_NORTH 10.0
#define BEACON_SPACING_EAST 20.0

// The centroid of the pattern can be moved using using the following parameters:
#define ORIGIN_OFFSET_NORTH 2.5 // shifts beacon pattern centroid North (m)
#define ORIGIN_OFFSET_EAST 5.0 // shifts beacon pattern centroid East (m)

enum class BeaconMeasurementMode : uint8_t {
    RANGE = 0,
    TDOA = 1,
};

static void offset_beacon_location(Location &beacon_loc, uint8_t beacon_id)
{
    switch (beacon_id) {
    case 0:
        // NE corner
        beacon_loc.offset(ORIGIN_OFFSET_NORTH + BEACON_SPACING_NORTH/2, ORIGIN_OFFSET_EAST + BEACON_SPACING_EAST/2);
        break;
    case 1:
        // SE corner
        beacon_loc.offset(ORIGIN_OFFSET_NORTH - BEACON_SPACING_NORTH/2, ORIGIN_OFFSET_EAST + BEACON_SPACING_EAST/2);
        break;
    case 2:
        // SW corner
        beacon_loc.offset(ORIGIN_OFFSET_NORTH - BEACON_SPACING_NORTH/2, ORIGIN_OFFSET_EAST - BEACON_SPACING_EAST/2);
        break;
    case 3:
        // NW corner
        beacon_loc.offset(ORIGIN_OFFSET_NORTH + BEACON_SPACING_NORTH/2, ORIGIN_OFFSET_EAST - BEACON_SPACING_EAST/2);
        break;
    }
}

static void tdoa_pair_for_index(uint8_t pair_index, uint8_t &anchor_id_a, uint8_t &anchor_id_b)
{
    uint8_t index = 0;
    for (uint8_t i = 0; i < NUM_BEACONS; i++) {
        for (uint8_t j = i + 1; j < NUM_BEACONS; j++) {
            if (index == pair_index) {
                anchor_id_a = i;
                anchor_id_b = j;
                return;
            }
            index++;
        }
    }

    anchor_id_a = 0;
    anchor_id_b = 1;
}

// constructor
AP_Beacon_SITL::AP_Beacon_SITL(AP_Beacon &frontend) :
    AP_Beacon_Backend(frontend),
    sitl(AP::sitl())
{
}

// return true if sensor is basically healthy (we are receiving data)
bool AP_Beacon_SITL::healthy()
{
    // healthy if we have parsed a message within the past 300ms
    return ((AP_HAL::millis() - last_update_ms) < AP_BEACON_TIMEOUT_MS);
}

// update the state of the sensor
void AP_Beacon_SITL::update(void)
{
    uint32_t now = AP_HAL::millis();
    if (now - last_update_ms < 10) {
        return;
    }

    uint8_t beacon_id = next_beacon;
    next_beacon = (next_beacon+1) % NUM_BEACONS;

    // truth location of the flight vehicle
    Location current_loc;
    current_loc.lat = sitl->state.latitude * 1.0e7f;
    current_loc.lng = sitl->state.longitude * 1.0e7f;
    current_loc.alt = sitl->state.altitude * 1.0e2;

    // where the beacon system origin is located
    Location beacon_origin;
    beacon_origin.lat = get_beacon_origin_lat() * 1.0e7f;
    beacon_origin.lng = get_beacon_origin_lon() * 1.0e7f;
    beacon_origin.alt = get_beacon_origin_alt() * 1.0e2;

    // position of each beacon
    Location beacon_loc = beacon_origin;
    offset_beacon_location(beacon_loc, beacon_id);

    const Vector2f beac_diff = beacon_origin.get_distance_NE(beacon_loc);
    const Vector2f veh_diff = beacon_origin.get_distance_NE(current_loc);

    Vector3f veh_pos3d(veh_diff.x, veh_diff.y, (beacon_origin.alt - current_loc.alt)*1.0e-2f);
    Vector3f beac_pos3d(beac_diff.x, beac_diff.y, (beacon_loc.alt - beacon_origin.alt)*1.0e-2f);
    Vector3f beac_veh_offset = veh_pos3d - beac_pos3d;

    const BeaconMeasurementMode mode = (BeaconMeasurementMode)get_sitl_measurement_mode();
    if (mode == BeaconMeasurementMode::TDOA) {
        Vector3f beacon_pos[NUM_BEACONS];
        for (uint8_t i = 0; i < NUM_BEACONS; i++) {
            Location loc = beacon_origin;
            offset_beacon_location(loc, i);
            const Vector2f diff = beacon_origin.get_distance_NE(loc);
            beacon_pos[i] = Vector3f(diff.x, diff.y, (loc.alt - beacon_origin.alt)*1.0e-2f);
            set_beacon_position(i, beacon_pos[i]);
        }

        constexpr uint8_t num_pairs = (NUM_BEACONS * (NUM_BEACONS - 1)) / 2;
        uint8_t anchor_id_a;
        uint8_t anchor_id_b;
        tdoa_pair_for_index(next_tdoa_pair, anchor_id_a, anchor_id_b);
        next_tdoa_pair = (next_tdoa_pair + 1) % num_pairs;

        const float distance_a = (veh_pos3d - beacon_pos[anchor_id_a]).length();
        const float distance_b = (veh_pos3d - beacon_pos[anchor_id_b]).length();
        set_tdoa_measurement(anchor_id_a, anchor_id_b, distance_b - distance_a, 0.15f);
    } else {
        set_beacon_position(beacon_id, beac_pos3d);
        set_beacon_distance(beacon_id, beac_veh_offset.length());
    }
    set_vehicle_position(veh_pos3d, 0.5f);
    last_update_ms = now;
}

#endif // AP_BEACON_SITL_ENABLED
