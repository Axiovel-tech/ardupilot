#include <AP_GPS/AP_GPS.h>
#include <GCS_MAVLink/GCS.h>

#include "AC_DroneShowManager.h"
#include "AC_DroneShowManager/DroneShow_Enums.h"

// Time before the start when a start scheduled on the show clock is frozen on
// the internal clock; the drone show mode calibrates the barometer at the
// same lead time before takeoff.
static constexpr int64_t SHOW_CLOCK_COMMIT_BEFORE_START_USEC = 10000000;

// The ground re-sends a schedule with drift corrections of milliseconds; a
// change this large under the same ID is its re-send in a new timebase that
// this drone's show clock has not stepped to yet
static constexpr uint64_t SHOW_SCHEDULE_MAX_CORRECTION_USEC = 1000000;

static uint64_t get_gps_timestamp_usec();
static bool is_safe_to_change_start_time_in_stage(DroneShowModeStage stage);

int64_t AC_DroneShowManager::get_elapsed_time_since_start_usec() const
{
    uint64_t now, reference, diff;
    
    if (uses_gps_time_for_show_start()) {
        now = get_gps_timestamp_usec();
        reference = _start_time_unix_usec;
    } else {
        now = AP_HAL::micros64();
        reference = _start_time_on_internal_clock_usec;
    }

    if (reference > 0) {
        if (reference > now) {
            diff = reference - now;
            if (diff < INT64_MAX) {
                return -diff;
            } else {
                return INT64_MIN;
            }
        } else if (reference < now) {
            diff = now - reference;
            if (diff < INT64_MAX) {
                return diff;
            } else {
                return INT64_MAX;
            }
        } else {
            return 0;
        }
    } else {
        return INT64_MIN;
    }
}

int32_t AC_DroneShowManager::get_elapsed_time_since_start_msec() const
{
    int64_t elapsed_usec = get_elapsed_time_since_start_usec();

    // Using -INFINITY here can lead to FPEs on macOS in the SITL simulator
    // when compiling in release mode, hence we use a large negative number
    // representing one day
    if (elapsed_usec <= -86400000000) {
        return -86400000;
    } else if (elapsed_usec >= 86400000000) {
        return 86400000;
    } else {
        return static_cast<int32_t>(elapsed_usec / 1000);
    }
}

float AC_DroneShowManager::get_elapsed_time_since_start_sec() const
{
    int64_t elapsed_usec = get_elapsed_time_since_start_usec();

    // Using -INFINITY here can lead to FPEs on macOS in the SITL simulator
    // when compiling in release mode, hence we use a large negative number
    // representing one day
    return elapsed_usec == INT64_MIN ? -86400 : static_cast<float>(elapsed_usec / 1000) / 1000.0f;
}

void AC_DroneShowManager::get_scene_index_and_show_clock_within_scene(
    ssize_t* scene_out, float* show_clock_sec_out
) const {
    sb_control_output_time_t time_info;
    ssize_t scene;
    float show_clock_sec;
    
    if (_stage_in_drone_show_mode != DroneShow_Performing) {
        scene = 0;
        show_clock_sec = 0;
    } else {
        time_info = sb_show_controller_get_current_output_time(&_show_controller);
        scene = time_info.scene;
        show_clock_sec = time_info.warped_time_in_scene_sec;
        
        if (scene < 0) {
            scene = sb_screenplay_size(&_screenplay);  // out of range value
        }
        if (scene >= 255) {
            scene = 255;  // out of range value or too many chapters
        }
    
        if (!isfinite(show_clock_sec)) {
            show_clock_sec = 0;
        }
    }
    
    if (scene_out) {
        *scene_out = scene;
    }
    
    if (show_clock_sec_out) {
        *show_clock_sec_out = show_clock_sec;
    }
}

int64_t AC_DroneShowManager::get_time_until_start_usec() const
{
    return -get_elapsed_time_since_start_usec();
}

float AC_DroneShowManager::get_time_until_start_sec() const
{
    return -get_elapsed_time_since_start_sec();
}

uint32_t AC_DroneShowManager::_get_gps_synced_timestamp_in_millis_for_lights() const
{
    // A start scheduled on the internal clock is the fleet's shared reference:
    // every drone schedules it from the same fleet-wide event (the RC start
    // switch flip, a GCS schedule or a timecode cue), so a start-anchored
    // phase agrees across drones to within that event's delivery jitter.
    // This branch comes FIRST, before the GPS-time branch: _is_gps_time_ok()
    // latches for the rest of the boot once a GPS ever delivered week+ToW,
    // so in a mixed fleet (some drones briefly saw GPS time, some did not) a
    // GPS-epoch phase would disagree with the millis() fallback of the
    // GPS-less drones. When the show runs on the internal clock, the
    // scheduled instant is the authoritative common timebase for everyone.
    // The GPS sync mode is excluded: there the start itself lives on the GPS
    // timebase, and without GPS time the internal reference would not apply.
    if (!uses_gps_time_for_show_start() && has_scheduled_start_time()) {
        int64_t elapsed_usec = get_elapsed_time_since_start_usec();
        if (elapsed_usec != INT64_MIN && elapsed_usec != INT64_MAX) {
            // Truncating the (possibly negative) elapsed time to uint32_t
            // yields a phase that still agrees across drones before the
            // start; the one small phase step at the sign wrap coincides
            // with the show starting, where the light program takes over
            // anyway.
            return static_cast<uint32_t>(static_cast<uint64_t>(elapsed_usec / 1000));
        }
    }

    // The show clock is the fleet-wide reference when we have one, so idle
    // signals blink in phase across the fleet even without a scheduled start.
    if (uses_show_clock_for_show_start() && _show_clock.is_valid()) {
        return _show_clock.local_to_show_usec(AP_HAL::micros64()) / 1000;
    }

    // No need to worry about loss of GPS fix; AP::gps().time_epoch_usec() is
    // smart enough to extrapolate from the timestamp of the latest fix.
    //
    // Also no need to worry about overflow; AP::gps().time_epoch_usec() / 1000
    // is too large for an uint32_t but it doesn't matter as we will truncate
    // the high bits.
    if (_is_gps_time_ok()) {
        return get_gps_timestamp_usec() / 1000;
    }

    return AP_HAL::millis();
}

bool AC_DroneShowManager::_is_gps_time_ok() const
{
    // AP::gos().time_week() starts from zero and gets set to a non-zero value
    // when we start receiving full time information from the GPS. It may happen
    // that the GPS subsystem receives iTOW information from the GPS module but
    // no week number; we deem this unreliable so we return false in this case.
    return AP::gps().time_week() > 0;
}

bool AC_DroneShowManager::_is_start_time_reference_ok() const
{
    return uses_show_clock_for_show_start() ? _show_clock.is_valid() : _is_gps_time_ok();
}

void AC_DroneShowManager::_set_start_time_on_show_clock(uint64_t start_cluster_usec)
{
    _start_time_on_show_clock_usec = start_cluster_usec;
    _start_time_on_internal_clock_usec = _show_clock.show_to_local_usec(start_cluster_usec);
    _start_time_requested_by = StartTimeSource::PARAMETER;
    _start_time_on_show_clock_committed = get_time_until_start_usec() <= SHOW_CLOCK_COMMIT_BEFORE_START_USEC;
    if (_start_time_on_show_clock_committed) {
        write_show_clock_commit_log_message();
    }
}

bool AC_DroneShowManager::_is_start_time_committed() const
{
    if (!has_scheduled_start_time()) {
        return false;
    }
    if (!_is_safe_to_change_start_time_in_current_stage()) {
        return true;
    }
    if (_start_time_on_show_clock_usec > 0 && !_start_time_on_show_clock_committed) {
        return false;
    }

    // A lights-only show runs on the ground in a stage that allows changes, so
    // the start stays committed until the end of the show
    const float duration_sec = _trajectory_stats.duration_sec;
    const int64_t duration_usec = isfinite(duration_sec) && duration_sec > 0 ? int64_t(duration_sec * 1.0e6f) : 0;
    return get_elapsed_time_since_start_usec() < duration_usec;
}

ShowScheduleAction AC_DroneShowManager::_apply_show_schedule(uint16_t schedule_id, uint64_t start_cluster_usec)
{
    const bool committed = _is_start_time_committed();

    // Nothing changes a show once it started, in the air or lights-only on
    // the ground
    if (!_is_safe_to_change_start_time_in_current_stage() || (committed && get_time_until_start_usec() <= 0)) {
        return ShowScheduleAction_Ignored;
    }

    if (start_cluster_usec == 0) {
        _show_schedule_id = schedule_id;
        if (!has_scheduled_start_time()) {
            return ShowScheduleAction_Cleared;
        }
        clear_scheduled_start_time(/* force = */ true);
        if (committed) {
            gcs().send_text(MAV_SEVERITY_WARNING, "Show start aborted");
            return ShowScheduleAction_Aborted;
        }
        return ShowScheduleAction_Cleared;
    }

    if (schedule_id == _show_schedule_id) {
        if (_start_time_on_show_clock_usec > 0 && !_start_time_on_show_clock_committed) {
            const uint64_t held_usec = _start_time_on_show_clock_usec;
            const uint64_t change_usec = start_cluster_usec > held_usec ? start_cluster_usec - held_usec : held_usec - start_cluster_usec;
            if (change_usec >= SHOW_SCHEDULE_MAX_CORRECTION_USEC) {
                // Taken once the show clock steps and drops the held start
                return ShowScheduleAction_Ignored;
            }
            // A drift correction of the start this drone holds
            if (_show_clock.is_valid()) {
                _set_start_time_on_show_clock(start_cluster_usec);
            }
            return ShowScheduleAction_Scheduled;
        }
        // Committed, refused, taken over by the RC switch, or a repetition
        // in the old timebase of a start that a timebase step dropped
        if (_dropped_start_cluster_usec == 0 || start_cluster_usec == _dropped_start_cluster_usec) {
            return ShowScheduleAction_Ignored;
        }
    }

    if (committed) {
        // The ground never moves a committed start, so a new schedule means
        // that this drone missed the abort that came before it
        write_show_schedule_log_message(_show_schedule_id, _start_time_on_show_clock_usec, ShowScheduleAction_Aborted);
        clear_scheduled_start_time(/* force = */ true);
        gcs().send_text(MAV_SEVERITY_WARNING, "Show start aborted");
    }

    if (!_show_clock.is_valid()) {
        // The ID is not recorded, so the next repetition counts as new once
        // the clock is back
        if (has_scheduled_start_time()) {
            clear_scheduled_start_time(/* force = */ true);
        }
        return ShowScheduleAction_NoClock;
    }

    _show_schedule_id = schedule_id;
    _dropped_start_cluster_usec = 0;

    const int64_t lead_usec = int64_t(_show_clock.show_to_local_usec(start_cluster_usec)) - int64_t(AP_HAL::micros64());
    if (lead_usec <= SHOW_CLOCK_COMMIT_BEFORE_START_USEC) {
        if (has_scheduled_start_time()) {
            clear_scheduled_start_time(/* force = */ true);
        }
        gcs().send_text(MAV_SEVERITY_WARNING, "Show start refused: less than 10 s ahead");
        return ShowScheduleAction_Refused;
    }

    _set_start_time_on_show_clock(start_cluster_usec);
    return ShowScheduleAction_Scheduled;
}

void AC_DroneShowManager::_update_start_time_from_show_clock()
{
    if (_start_time_on_show_clock_usec == 0 || _start_time_on_show_clock_committed) {
        return;
    }

    // The drone show mode runs more often than this update and may already
    // have acted on a start within the commit lead (barometer calibration),
    // so such a start is frozen as it is. Without fresh samples we keep the
    // last mapping and still commit on time.
    if (get_time_until_start_usec() > SHOW_CLOCK_COMMIT_BEFORE_START_USEC && _show_clock.is_valid()) {
        _start_time_on_internal_clock_usec = _show_clock.show_to_local_usec(_start_time_on_show_clock_usec);
    }

    _start_time_on_show_clock_committed = get_time_until_start_usec() <= SHOW_CLOCK_COMMIT_BEFORE_START_USEC;
    if (_start_time_on_show_clock_committed) {
        write_show_clock_commit_log_message();
    }
}

AC_DroneShowManager::ShowClock::ShowClock() :
    _window(),
    _count(0),
    _next(0),
    _offset_usec(0),
    _step_window(),
    _step_count(0),
    _last_sample_at_msec(0)
{
}

AC_DroneShowManager::ShowClock::SampleResult AC_DroneShowManager::ShowClock::handle_sample(uint64_t show_time_usec, uint64_t local_usec)
{
    const int64_t offset_usec = int64_t(local_usec) - int64_t(show_time_usec);
    const uint32_t now_msec = AP_HAL::millis();
    const int64_t jump_usec = offset_usec - _offset_usec;
    const bool timebase_jump = jump_usec >= TIMEBASE_STEP_USEC || jump_usec <= -TIMEBASE_STEP_USEC;
    bool timebase_step = false;

    if (_count > 0 && now_msec - _last_sample_at_msec >= TIMEOUT_MSEC) {
        timebase_step = timebase_jump;
        _count = 0;
        _next = 0;
        _step_count = 0;
    }

    if (_count > 0 && (jump_usec < -MAX_AHEAD_USEC || jump_usec > MAX_BEHIND_USEC)) {
        const int64_t step_spread_usec = offset_usec - _step_window[0];
        if (_step_count == 0 || step_spread_usec > STEP_AGREE_USEC || step_spread_usec < -STEP_AGREE_USEC) {
            _step_count = 0;
        }
        _step_window[_step_count++] = offset_usec;
        if (_step_count < STEP_CONFIRM_SAMPLES) {
            return SampleResult::IGNORED;
        }
        timebase_step = timebase_jump;
        _count = 0;
        _next = 0;
        for (uint8_t i = 0; i < STEP_CONFIRM_SAMPLES - 1; i++) {
            push(_step_window[i]);
        }
    }
    _step_count = 0;

    push(offset_usec);
    _last_sample_at_msec = now_msec;
    return timebase_step ? SampleResult::TIMEBASE_STEP : SampleResult::ACCEPTED;
}

void AC_DroneShowManager::ShowClock::push(int64_t offset_usec)
{
    _window[_next] = offset_usec;
    _next = (_next + 1) % WINDOW_SAMPLES;
    if (_count < WINDOW_SAMPLES) {
        _count++;
    }

    _offset_usec = _window[0];
    for (uint8_t i = 1; i < _count; i++) {
        _offset_usec = MIN(_offset_usec, _window[i]);
    }
}

bool AC_DroneShowManager::ShowClock::is_valid() const
{
    return _count >= MIN_VALID_SAMPLES && AP_HAL::millis() - _last_sample_at_msec < TIMEOUT_MSEC;
}

uint64_t AC_DroneShowManager::ShowClock::local_to_show_usec(uint64_t local_usec) const
{
    return local_usec - _offset_usec;
}

uint64_t AC_DroneShowManager::ShowClock::show_to_local_usec(uint64_t show_usec) const
{
    return show_usec + _offset_usec;
}

bool AC_DroneShowManager::_is_safe_to_change_start_time_in_current_stage() const {
    return is_safe_to_change_start_time_in_stage(_stage_in_drone_show_mode);
}

static bool is_safe_to_change_start_time_in_stage(DroneShowModeStage stage) {
    return (
        stage == DroneShow_Off ||
        stage == DroneShow_Init ||
        stage == DroneShow_WaitForStartTime ||
        stage == DroneShow_Landed
    );
}

// Returns the current time according to the GPS, in microseconds.
//
// This function takes care of eliminating glitches in the GPS timestamp that
// may happen when a GPS message updates the stored GPS time-of-week but not
// the correspnding GPS week number. This is a glitch that is known to have
// happened with U-blox GPS modules in ArduPilot 4.6, but other versions or
// GPS drivers may also be affected so we try to protect against it.
//
// The fix we use here is simply to assume that GPS time cannot move backward.
// If we receive a reported GPS time that is earlier than the previous value
// (which we store here), we return the previous value instead.
//
// We assume that glitches only occur "backwards" in time, not forward. That
// would require a GPS message handler that updates the GPS week number _without_
// updating the GPS time-of-week, which is unlikely to happen in practice
// (all GPS messages that carry the week number are also likely to carry the
// time-of-week).
static uint64_t get_gps_timestamp_usec()
{
    // AP::gps().time_epoch_usec() is smart enough to handle the case when
    // the GPS fix was lost so no need to worry about loss of GPS fix here.
    static uint64_t last_gps_time_usec = 0;
    uint64_t current_gps_time_usec = AP::gps().time_epoch_usec();

    if (current_gps_time_usec < last_gps_time_usec) {
        return last_gps_time_usec;
    } else {
        last_gps_time_usec = current_gps_time_usec;
        return current_gps_time_usec;
    }
}
