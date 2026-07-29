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
#pragma once

#include "AP_VisualOdom_config.h"

#if HAL_VISUALODOM_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <GCS_MAVLink/GCS_config.h>
#if HAL_GCS_ENABLED
#include <GCS_MAVLink/GCS_MAVLink.h>
#endif
#include <AP_Math/AP_Math.h>

// EKF3 historically clamped the external-nav vertical observation noise to this
// minimum.  Retained as the implicit vertical floor when VISO_ALT_M_NSE is unset,
// so that leaving the new parameter at zero reproduces the old behaviour exactly.
#define EXTNAV_LEGACY_ALT_NOISE_FLOOR 0.1f

class AP_VisualOdom_Backend;

#define AP_VISUALODOM_TIMEOUT_MS 300

class AP_VisualOdom
{
public:

    // Unique terms of the symmetric 3x3 position covariance supplied by a
    // pose source.  Values are variances/covariances in m^2 in the same frame
    // as the reported position.
    struct PositionCovariance {
        float xx = NAN;
        float xy = NAN;
        float xz = NAN;
        float yy = NAN;
        float yz = NAN;
        float zz = NAN;

        bool has_valid_diagonal() const {
            return isfinite(xx) && isfinite(yy) && isfinite(zz) &&
                   xx >= 0.0f && yy >= 0.0f && zz >= 0.0f;
        }
    };

    AP_VisualOdom();

    // get singleton instance
    static AP_VisualOdom *get_singleton() {
        return _singleton;
    }

    // external position backend types (used by _TYPE parameter)
    enum class VisualOdom_Type {
        None         = 0,
#if AP_VISUALODOM_MAV_ENABLED
        MAV          = 1,
#endif
#if AP_VISUALODOM_INTELT265_ENABLED
        IntelT265    = 2,
        VOXL         = 3,
#endif
    };

    // detect and initialise any sensors
    void init();

    // return true if sensor is enabled
    bool enabled() const;

    // return true if sensor is basically healthy (we are receiving data)
    bool healthy() const;

    // get user defined orientation
    enum Rotation get_orientation() const { return (enum Rotation)_orientation.get(); }

    // get user defined scaling applied to position estimates
    float get_pos_scale() const { return _pos_scale; }

    // return a 3D vector defining the position offset of the camera in meters relative to the body frame origin
    const Vector3f &get_pos_offset(void) const { return _pos_offset; }

    // return the sensor delay in milliseconds (see _DELAY_MS parameter)
    uint16_t get_delay_ms() const { return MAX(0, _delay_ms); }

    // return velocity measurement noise in m/s
    float get_vel_noise() const { return _vel_noise; }
    
    // return horizontal position measurement noise in m
    float get_pos_noise() const { return _pos_noise; }

    // return vertical position measurement noise in m.
    //
    // When _ALT_M_NSE is left at zero this reproduces the historical behaviour
    // exactly: a single scalar served both axes, and EKF3 then clamped the
    // vertical observation noise to a minimum of 0.1m.  Folding that legacy
    // floor in here keeps the default bit-identical even though EKF3's own
    // vertical clamp has since been lowered to match the horizontal one.
    float get_alt_noise() const {
        if (is_positive(_alt_noise)) {
            return _alt_noise.get();
        }
        return MAX(_pos_noise.get(), EXTNAV_LEGACY_ALT_NOISE_FLOOR);
    }

    // ratio of vertical to horizontal position noise, used to keep the two axes
    // separated when the sensor reports its own error rather than relying on
    // the configured floors.
    //
    // Returns exactly 1 when _ALT_M_NSE is unset so that the vertical error
    // tracks the horizontal one identically to the old single-scalar path; the
    // legacy 0.1m floor folded into get_alt_noise() must not be allowed to leak
    // in here as a scale factor.
    float get_alt_noise_ratio() const {
        if (!is_positive(_alt_noise)) {
            return 1.0f;
        }
        const float pos_nse = _pos_noise.get();
        if (!is_positive(pos_nse)) {
            return 1.0f;
        }
        return _alt_noise.get() / pos_nse;
    }

    // return yaw measurement noise in rad
    float get_yaw_noise() const { return _yaw_noise; }

    // return quality threshold
    int8_t get_quality_min() const { return _quality_min; }

    // return quality as a measure from -1 ~ 100
    // -1 means failed, 0 means unknown, 1 is worst, 100 is best
    int8_t quality() const;

#if HAL_GCS_ENABLED
    // consume vision_position_delta mavlink messages
    void handle_vision_position_delta_msg(const mavlink_message_t &msg);
#endif

    // general purpose methods to consume position estimate data and send to EKF
    // distances in meters, roll, pitch and yaw are in radians
    // quality of -1 means failed, 0 means unknown, 1 is worst, 100 is best
    void handle_pose_estimate(uint64_t remote_time_us, uint32_t time_ms, float x, float y, float z, float roll, float pitch, float yaw, float posErr, float angErr, uint8_t reset_counter, int8_t quality, const PositionCovariance *pos_covariance = nullptr);
    void handle_pose_estimate(uint64_t remote_time_us, uint32_t time_ms, float x, float y, float z, const Quaternion &attitude, float posErr, float angErr, uint8_t reset_counter, int8_t quality, const PositionCovariance *pos_covariance = nullptr);
    
    // general purpose methods to consume velocity estimate data and send to EKF
    // velocity in NED meters per second
    // quality of -1 means failed, 0 means unknown, 1 is worst, 100 is best
    void handle_vision_speed_estimate(uint64_t remote_time_us, uint32_t time_ms, const Vector3f &vel, uint8_t reset_counter, int8_t quality);

    // request sensor's yaw be aligned with vehicle's AHRS/EKF attitude
    void request_align_yaw_to_ahrs();

    // update position offsets to align to AHRS position
    // should only be called when this library is not being used as the position source
    void align_position_to_ahrs(bool align_xy, bool align_z);

    // returns false if we fail arming checks, in which case the buffer will be populated with a failure message
    bool pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const;

    static const struct AP_Param::GroupInfo var_info[];

    VisualOdom_Type get_type(void) const {
        return _type;
    }

private:

    static AP_VisualOdom *_singleton;

    // parameters
    AP_Enum<VisualOdom_Type> _type; // sensor type
    AP_Vector3f _pos_offset;    // position offset of the camera in the body frame
    AP_Int8 _orientation;       // camera orientation on vehicle frame
    AP_Float _pos_scale;        // position scale factor applied to sensor values
    AP_Int16 _delay_ms;         // average delay relative to inertial measurements
    AP_Float _vel_noise;        // velocity measurement noise in m/s
    AP_Float _pos_noise;        // horizontal position measurement noise in meters
    AP_Float _alt_noise;        // vertical position measurement noise in meters (0 = use _pos_noise)
    AP_Float _yaw_noise;        // yaw measurement noise in radians
    AP_Int8 _quality_min;       // positions and velocities will only be sent to EKF if over this value.  if 0 all values sent to EKF

    // reference to backends
    AP_VisualOdom_Backend *_driver;
};

namespace AP {
    AP_VisualOdom *visualodom();
};

#endif // HAL_VISUALODOM_ENABLED
