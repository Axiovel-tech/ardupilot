#include "AC_DroneShowManager.h"

#include <AP_AHRS/AP_AHRS.h>
#include <GCS_MAVLink/GCS.h>

bool AC_DroneShowManager::get_global_takeoff_position(Location& loc) const
{
    // This function may be called any time, not only during the show, so we
    // need to take the parameters provided by the user, convert them into a
    // ShowCoordinateSystem object, and then use that to get the GPS coordinates
    sb_vector3_t vec;

    if (!_tentative_show_coordinate_system.is_valid())
    {
        return false;
    }

    vec.x = _takeoff_position_mm.x;
    vec.y = _takeoff_position_mm.y;
    vec.z = _takeoff_position_mm.z;

    _tentative_show_coordinate_system.convert_show_to_global_coordinate(vec, loc);

    return true;
}

float AC_DroneShowManager::get_motor_spool_up_time_sec() const {
    float value = 0.0f;

    if (AP_Param::get("MOT_SPOOL_TIME", value)) {
        if (value >= 0.0f) {
            return value;
        }
    }

    return DEFAULT_MOTOR_SPOOL_UP_TIME_SEC;
}

float AC_DroneShowManager::get_takeoff_speed_m_sec() const {
    float result = _wp_nav ? _wp_nav->get_default_speed_up_ms() : 0;
    if (result <= 0 || !isfinite(result)) {
        /* safety check */
        result = DEFAULT_TAKEOFF_SPEED_METERS_PER_SEC;
    }
    return result;
}

float AC_DroneShowManager::get_time_until_takeoff_sec()
{
    if (isnan(_projected_wall_clock_time_at_takeoff_sec)) {
        _projected_wall_clock_time_at_takeoff_sec = sb_screenplay_get_time_sec_for_scene_tag_and_warped_time_in_scene(
            &_screenplay, SceneTag_MainShow, _trajectory_stats.takeoff_time_sec
        );
        
        if (isnan(_projected_wall_clock_time_at_takeoff_sec)) {
            // This should not happen, but if it does, we just use infinity
            _projected_wall_clock_time_at_takeoff_sec = INFINITY;
        }
    }

    return get_time_until_start_sec() + _projected_wall_clock_time_at_takeoff_sec;
}

bool AC_DroneShowManager::is_prepared_to_take_off() const
{
    return !(_preflight_check_failures & ~DroneShowPreflightCheck_BadEstimateOnPad) &&
           (_is_gps_time_ok() || !uses_gps_time_for_show_start());
}

bool AC_DroneShowManager::notify_takeoff_attempt()
{
    if (
        _stage_in_drone_show_mode == DroneShow_WaitForStartTime && !_pad_check.has_samples &&
        (is_positive(_params.max_pad_speed_error_m_s) || is_positive(_params.max_pad_height_error_m))
    )
    {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "Takeoff refused, no pad check yet");
        return false;
    }

    if (_pad_check.failures)
    {
        gcs().send_text(
            MAV_SEVERITY_CRITICAL, "Takeoff refused, lane %u on pad: %.2f m/s, %.2f m",
            static_cast<unsigned>(_pad_check.lane),
            static_cast<double>(_pad_check.speed), static_cast<double>(_pad_check.height_error)
        );
        return false;
    }

    if (!is_prepared_to_take_off())
    {
        return false;
    }
    
    if (!_copy_show_coordinate_system_from_parameters_to(_show_coordinate_system))
    {
        return false;
    }

    // If the trajectory is circular (i.e. drone is supposed to land where it
    // took off from), tweak the end of the trajectory to account for placement
    // inaccuracies (we want to land where we took off from, not where we
    // _should_ have taken off from in a perfect world).
    //
    // This correction is nice to have but is not crucial. If an error happens
    // in the process below, we just bail out and proceed without the correction.
    if (
        _has_option(DroneShowOption_CorrectLandingPositionForCircularTrajectories) &&
        _trajectory_is_circular && !_trajectory_modified_for_landing
    ) {
        Location takeoff_location;
        sb_trajectory_t* trajectory;
        sb_vector3_t end;
        float land_speed_mm_s;

        if (!get_current_location(takeoff_location))
        {
            goto exit;
        }

        _show_coordinate_system.convert_global_to_show_coordinate(takeoff_location, end);
        
        // Get a handle to the current trajectory from the show controller so we can
        // modify its end point
        trajectory = sb_screenplay_scene_get_trajectory(&_main_show_scene);
        if (trajectory != nullptr)
        {
            land_speed_mm_s = get_landing_speed_m_sec() * 1000.0f;   /* [mm/s] */
            if (sb_trajectory_replace_end_to_land_at(trajectory, &_trajectory_stats, end, land_speed_mm_s)) {
                goto exit;
            }

            _trajectory_modified_for_landing = true;
        }
    }

exit:
    return true;
}

bool AC_DroneShowManager::_is_at_takeoff_position_xy(float xy_threshold) const
{
    Location takeoff_loc;
    
    if (!_tentative_show_coordinate_system.is_valid())
    {
        // User did not set up the takeoff position yet
        return false;
    }

    if (!get_global_takeoff_position(takeoff_loc))
    {
        // Show coordinate system not set up yet
        return false;
    }

    return _is_close_to_position(
        takeoff_loc,
        xy_threshold > 0 ? xy_threshold : _params.max_xy_placement_error_m,
        0
    );
}

void AC_DroneShowManager::_reset_pad_check()
{
    for (auto &samples : _pad_samples) {
        samples = {};
    }
    _pad_check = {};
    _pad_check.height_error = NAN;
}

void AC_DroneShowManager::_sample_pad_estimate()
{
    AP_AHRS &ahrs = AP::ahrs();
    Vector3f velocity;
    Location takeoff_loc;

    const bool has_takeoff_loc = get_global_takeoff_position(takeoff_loc);

#if HAL_NAVEKF3_AVAILABLE
    // A lane that is not primary on the pad may become primary in flight,
    // so every lane must be at rest
    if (ahrs.configured_ekf_type() == AP_AHRS::EKFType::THREE && ahrs.EKF3.activeCores() > 0) {
        int32_t takeoff_alt_cm;
        const bool has_takeoff_alt = has_takeoff_loc &&
            takeoff_loc.get_alt_cm(Location::AltFrame::ABOVE_ORIGIN, takeoff_alt_cm);

        for (uint8_t i = 0; i < MIN(ahrs.EKF3.activeCores(), MAX_EKF_CORES); i++) {
            postype_t pos_d;

            ahrs.EKF3.getVelNED(i, velocity);
            _pad_samples[i].velocity_sum += velocity;
            _pad_samples[i].max_speed = MAX(_pad_samples[i].max_speed, velocity.length());
            _pad_samples[i].velocity_count++;

            if (has_takeoff_alt && ahrs.EKF3.getPosD(i, pos_d)) {
                _pad_samples[i].height_sum += -pos_d - takeoff_alt_cm * 0.01;
                _pad_samples[i].height_count++;
            }
        }
        return;
    }
#endif

    Location current_loc;
    ftype height;

    if (ahrs.get_velocity_NED(velocity)) {
        _pad_samples[0].velocity_sum += velocity;
        _pad_samples[0].max_speed = MAX(_pad_samples[0].max_speed, velocity.length());
        _pad_samples[0].velocity_count++;
    }

    if (
        has_takeoff_loc &&
        get_current_location(current_loc) &&
        current_loc.get_height_above(takeoff_loc, height)
    ) {
        _pad_samples[0].height_sum += height;
        _pad_samples[0].height_count++;
    }
}

void AC_DroneShowManager::_update_pad_check()
{
    const bool failed_before = _pad_check.failures != 0;
    float worst_score = -1;

    _pad_check = {};
    _pad_check.height_error = NAN;

    for (uint8_t i = 0; i < MAX_EKF_CORES; i++) {
        const auto &samples = _pad_samples[i];
        float speed = 0, height_error = NAN, score = 0;
        uint8_t failures = 0;

        if (samples.velocity_count == 0 && samples.height_count == 0) {
            continue;
        }

        _pad_check.has_samples = true;

        // The truth on the pad is zero velocity, so a biased estimate shows
        // in the mean while the noise averages out. A non-finite estimate
        // fails.
        if (samples.velocity_count > 0) {
            speed = (samples.velocity_sum / samples.velocity_count).length();
            if (is_positive(_params.max_pad_speed_error_m_s)) {
                const float speed_score = isfinite(speed) ?
                    speed / _params.max_pad_speed_error_m_s : INFINITY;
                if (speed_score > 1) {
                    failures |= DroneShowPadCheck_Speed;
                }
                score = speed_score;
            }
        }
        if (samples.height_count > 0) {
            height_error = samples.height_sum / samples.height_count;
            if (is_positive(_params.max_pad_height_error_m)) {
                const float height_score = isfinite(height_error) ?
                    fabsf(height_error) / _params.max_pad_height_error_m : INFINITY;
                if (height_score > 1) {
                    failures |= DroneShowPadCheck_Height;
                }
                score = MAX(score, height_score);
            }
        }

        if (score > worst_score) {
            worst_score = score;
            _pad_check.speed = speed;
            _pad_check.max_speed = samples.max_speed;
            _pad_check.height_error = height_error;
            _pad_check.failures = failures;
            _pad_check.lane = i;
        }
    }

    if (failed_before != (_pad_check.failures != 0)) {
        gcs().send_text(
            _pad_check.failures ? MAV_SEVERITY_WARNING : MAV_SEVERITY_INFO,
            "Pad check %s: lane %u, %.2f m/s, %.2f m", _pad_check.failures ? "failed" : "passed",
            static_cast<unsigned>(_pad_check.lane),
            static_cast<double>(_pad_check.speed), static_cast<double>(_pad_check.height_error)
        );
    }

    for (auto &samples : _pad_samples) {
        samples = {};
    }
}
