#include <AP_GPS/AP_GPS.h>

#include "AC_DroneShowManager.h"
#include "AC_DroneShowManager/DroneShow_Enums.h"

namespace {

constexpr uint8_t UWB_SHOW_SYNC_PAYLOAD_SIZE = 18;
constexpr uint8_t UWB_SHOW_SYNC_MIN_CONSISTENT_SAMPLES = 3;
constexpr uint32_t UWB_SHOW_SYNC_TIMEOUT_MSEC = 1000;
constexpr uint64_t UWB_SHOW_SYNC_MAX_DEADLINE_JITTER_USEC = 20000;
constexpr float UWB_SHOW_SYNC_COMMIT_BEFORE_TAKEOFF_SEC = 12.0f;

// The SR250 clock tick is 15.65 ps. Splitting the division avoids overflowing
// uint64_t for deadlines up to one GPS week away.
constexpr uint64_t UWB_TICK_TO_USEC_NUMERATOR = 1565;
constexpr uint64_t UWB_TICK_TO_USEC_DENOMINATOR = 100000000;
constexpr uint64_t UWB_SHOW_SYNC_MAX_DELTA_TICKS = 38600000000000000ULL;

uint64_t read_le64(const uint8_t* data)
{
    uint64_t value = 0;
    for (uint8_t i = 0; i < 8; i++) {
        value |= static_cast<uint64_t>(data[i]) << (8 * i);
    }
    return value;
}

uint16_t read_le16(const uint8_t* data)
{
    return static_cast<uint16_t>(data[0]) |
           (static_cast<uint16_t>(data[1]) << 8);
}

uint64_t uwb_ticks_to_usec(uint64_t ticks)
{
    const uint64_t whole = ticks / UWB_TICK_TO_USEC_DENOMINATOR;
    const uint64_t remainder = ticks % UWB_TICK_TO_USEC_DENOMINATOR;
    return whole * UWB_TICK_TO_USEC_NUMERATOR +
           (remainder * UWB_TICK_TO_USEC_NUMERATOR + UWB_TICK_TO_USEC_DENOMINATOR / 2) /
               UWB_TICK_TO_USEC_DENOMINATOR;
}

bool generation_is_newer(uint16_t candidate, uint16_t current)
{
    return static_cast<int16_t>(candidate - current) > 0;
}

} // namespace

void AC_DroneShowManager::_reset_uwb_show_sync()
{
    if (_start_time_requested_by == StartTimeSource::UWB_LTC) {
        _start_time_on_internal_clock_usec = 0;
        _start_time_requested_by = StartTimeSource::NONE;
    }

    _uwb_show_sync = {};
}

bool AC_DroneShowManager::_handle_uwb_show_sync_message(const uint8_t* data, uint8_t length)
{
    if (length != UWB_SHOW_SYNC_PAYLOAD_SIZE ||
        _params.time_sync_mode != TimeSyncMode_UWBLTC) {
        return false;
    }

    // An RC fallback owns the schedule until this run is cleared. A committed
    // UWB deadline is similarly immutable; cancellation after commit belongs
    // to the explicit show-abort path, not clock synchronization.
    if (_start_time_requested_by == StartTimeSource::RC_SWITCH ||
        _uwb_show_sync.committed ||
        _stage_in_drone_show_mode != DroneShow_WaitForStartTime) {
        return true;
    }

    const uint64_t cluster_now_tick = read_le64(data);
    const uint64_t cluster_start_tick = read_le64(data + 8);
    const uint16_t generation = read_le16(data + 16);
    const uint32_t now_msec = AP_HAL::millis();

    // Generation ordering is meaningful only while the source is fresh. Once
    // it expires, discard history so a restarted Anchor 0 can begin again at
    // any generation; expired deadlines are rejected independently below.
    if (_uwb_show_sync.have_generation && _uwb_show_sync.last_message_msec != 0 &&
        now_msec - _uwb_show_sync.last_message_msec > UWB_SHOW_SYNC_TIMEOUT_MSEC) {
        _reset_uwb_show_sync();
    }

    // An expired deadline must never turn into an immediate or late show
    // start. Anchor rescheduling is represented by a newer generation.
    uint64_t delta_ticks = 0;
    if (cluster_start_tick != 0) {
        if (cluster_start_tick <= cluster_now_tick) {
            return true;
        }
        delta_ticks = cluster_start_tick - cluster_now_tick;
        if (delta_ticks > UWB_SHOW_SYNC_MAX_DELTA_TICKS) {
            return true;
        }
    }

    const bool new_generation = !_uwb_show_sync.have_generation ||
        generation_is_newer(generation, _uwb_show_sync.generation);
    if (_uwb_show_sync.have_generation && generation != _uwb_show_sync.generation &&
        !new_generation) {
        return true;
    }

    if (cluster_start_tick == 0) {
        if (new_generation || generation == _uwb_show_sync.generation) {
            _reset_uwb_show_sync();
            _uwb_show_sync.have_generation = true;
            _uwb_show_sync.generation = generation;
            _uwb_show_sync.last_message_msec = now_msec;
        }
        return true;
    }

    if (new_generation) {
        _reset_uwb_show_sync();
        _uwb_show_sync.have_generation = true;
        _uwb_show_sync.generation = generation;
        _uwb_show_sync.cluster_start_tick = cluster_start_tick;
    } else if (cluster_start_tick != _uwb_show_sync.cluster_start_tick) {
        // A deadline is immutable within one generation. Rescheduling requires
        // a new generation so delayed packets cannot move an armed show.
        return true;
    }

    if (_uwb_show_sync.last_cluster_tick != 0 &&
        cluster_now_tick <= _uwb_show_sync.last_cluster_tick) {
        return true;
    }

    const uint64_t delta_usec = uwb_ticks_to_usec(delta_ticks);
    const uint64_t now_usec = AP_HAL::micros64();
    if (delta_usec > UINT64_MAX - now_usec) {
        return true;
    }
    const uint64_t candidate_deadline_usec = now_usec + delta_usec;

    _uwb_show_sync.last_cluster_tick = cluster_now_tick;
    _uwb_show_sync.last_message_msec = now_msec;

    if (_uwb_show_sync.consistent_samples == 0) {
        _uwb_show_sync.best_internal_deadline_usec = candidate_deadline_usec;
        _uwb_show_sync.consistent_samples = 1;
    } else {
        const uint64_t best = _uwb_show_sync.best_internal_deadline_usec;
        const uint64_t difference = candidate_deadline_usec > best
            ? candidate_deadline_usec - best
            : best - candidate_deadline_usec;

        if (difference <= UWB_SHOW_SYNC_MAX_DEADLINE_JITTER_USEC) {
            // Transport latency can only make the observed internal deadline
            // later, so retain the earliest consistent observation.
            if (candidate_deadline_usec < best) {
                _uwb_show_sync.best_internal_deadline_usec = candidate_deadline_usec;
            }
            if (_uwb_show_sync.consistent_samples < UINT8_MAX) {
                _uwb_show_sync.consistent_samples++;
            }
        } else if (candidate_deadline_usec < best) {
            // A substantially earlier group may be a better low-latency sample.
            // Reacquire before allowing it to move the schedule.
            _uwb_show_sync.best_internal_deadline_usec = candidate_deadline_usec;
            _uwb_show_sync.consistent_samples = 1;
            _uwb_show_sync.locked = false;
            if (_start_time_requested_by == StartTimeSource::UWB_LTC) {
                _start_time_on_internal_clock_usec = 0;
                _start_time_requested_by = StartTimeSource::NONE;
            }
        }
    }

    if (_uwb_show_sync.consistent_samples >= UWB_SHOW_SYNC_MIN_CONSISTENT_SAMPLES) {
        _uwb_show_sync.locked = true;
        _start_time_on_internal_clock_usec = _uwb_show_sync.best_internal_deadline_usec;
        _start_time_requested_by = StartTimeSource::UWB_LTC;
    }

    return true;
}

void AC_DroneShowManager::_update_uwb_show_sync()
{
    if (_start_time_requested_by != StartTimeSource::UWB_LTC ||
        !_uwb_show_sync.locked || _uwb_show_sync.committed) {
        return;
    }

    if (AP_HAL::millis() - _uwb_show_sync.last_message_msec > UWB_SHOW_SYNC_TIMEOUT_MSEC) {
        _reset_uwb_show_sync();
        return;
    }

    const float time_until_takeoff_sec = get_time_until_takeoff_sec();
    if (isfinite(time_until_takeoff_sec) &&
        time_until_takeoff_sec <= UWB_SHOW_SYNC_COMMIT_BEFORE_TAKEOFF_SEC) {
        _uwb_show_sync.committed = true;
    }
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
    // No need to worry about loss of GPS fix; AP::gps().time_epoch_usec() is
    // smart enough to extrapolate from the timestamp of the latest fix.
    //
    // Also no need to worry about overflow; AP::gps().time_epoch_usec() / 1000
    // is too large for an uint32_t but it doesn't matter as we will truncate
    // the high bits.
    if (_is_gps_time_ok()) {
        return get_gps_timestamp_usec() / 1000;
    } else {
        return AP_HAL::millis();
    }
}

bool AC_DroneShowManager::_is_gps_time_ok() const
{
    // AP::gos().time_week() starts from zero and gets set to a non-zero value
    // when we start receiving full time information from the GPS. It may happen
    // that the GPS subsystem receives iTOW information from the GPS module but
    // no week number; we deem this unreliable so we return false in this case.
    return AP::gps().time_week() > 0;
}
