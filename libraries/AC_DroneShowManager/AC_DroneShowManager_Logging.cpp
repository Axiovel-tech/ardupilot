#include "AC_DroneShowManager.h"

#include <AP_Logger/AP_Logger.h>
#include <skybrush/skybrush.h>

// Write a drone show status log entry
void AC_DroneShowManager::write_show_status_log_message() const
{
    sb_rgb_color_t color;
    Vector3f dist;
    ssize_t scene;
    float show_clock_sec;
    const sb_control_output_t* output = sb_show_controller_get_current_output(&_show_controller);
    sb_vector3_t position;
    
    if (is_gdmb_show_loaded() && _gdmb_reference_valid) {
        position = {
            _gdmb_reference.pos_m.x * 1000.0f, _gdmb_reference.pos_m.y * 1000.0f, _gdmb_reference.pos_m.z * 1000.0f
        };
    } else if (!sb_control_output_get_position_if_set(output, &position)) {
        memset(&position, 0, sizeof(position));
    }
    
    get_last_rgb_led_color(color);
    get_distance_from_desired_position(dist);
    get_scene_index_and_show_clock_within_scene(&scene, &show_clock_sec);
    
    const struct log_DroneShowStatus pkt {
        LOG_PACKET_HEADER_INIT(LOG_DRONE_SHOW_MSG),
        time_us         : AP_HAL::micros64(),
        wall_clock_ms   : get_elapsed_time_since_start_msec(),
        stage           : static_cast<uint8_t>(get_stage_in_drone_show_mode()),
        scene           : static_cast<uint8_t>(scene),
        show_clock_ms   : static_cast<int32_t>(show_clock_sec * 1000.0f),
        x               : position.x / 1000.0f,   // convert from millimeters to meters
        y               : position.y / 1000.0f,   // convert from millimeters to meters
        z               : position.z / 1000.0f,   // convert from millimeters to meters
        red             : color.red,
        green           : color.green,
        blue            : color.blue,
        h_dist          : hypotf(dist.x, dist.y),
        v_dist          : dist.z,
    };

    AP::logger().WriteBlock(&pkt, sizeof(pkt));
}

// Write a log entry comparing the show reference with the estimated position.
//
// Everything is in NEU metres relative to the EKF origin, computed without
// the centimetre rounding of Location, so that the difference of the two
// positions is the tracking error.
void AC_DroneShowManager::write_show_reference_log_message(float time)
{
    GdmbReference ref;
    sb_vector3_t show_pos_mm;
    uint8_t source;
    int64_t show_time_us;
    Vector3f vel_cms, acc_cmss, actual_ned_m, origin_neu_m;
    Location origin;

    if (_get_gdmb_reference_at_seconds(time, ref)) {
        source = 1;
        show_time_us = ref.show_time_us;
        show_pos_mm = { ref.pos_m.x * 1000.0f, ref.pos_m.y * 1000.0f, ref.pos_m.z * 1000.0f };
    } else {
        const sb_control_output_t* output = _get_raw_show_control_output_at_seconds(time);
        if (!output || !sb_control_output_get_position_if_set(output, &show_pos_mm)) {
            return;
        }
        source = 0;
        show_time_us = static_cast<int64_t>(
            sb_show_controller_get_current_output_time(&_show_controller).warped_time_in_scene_sec * 1.0e6f
        );
    }

    if (!get_desired_velocity_neu_in_cms_per_seconds_at_seconds(time, vel_cms)) {
        vel_cms.zero();
    }
    if (!get_desired_acceleration_neu_in_cms_per_seconds_squared_at_seconds(time, acc_cmss)) {
        acc_cmss.zero();
    }
    if (!get_current_relative_position_NED_origin(actual_ned_m)) {
        return;
    }

    // Show origin relative to the EKF origin, as in convert_show_to_global_coordinate()
    origin.zero();
    origin.lat = _show_coordinate_system.origin_lat;
    origin.lng = _show_coordinate_system.origin_lng;
    if (_show_coordinate_system.origin_amsl_valid) {
        origin.set_alt_cm(_show_coordinate_system.origin_amsl_mm / 10.0, Location::AltFrame::ABSOLUTE);
    } else {
        origin.set_alt_cm(0, Location::AltFrame::ABOVE_HOME);
    }
    if (!origin.get_vector_from_origin_NEU_m(origin_neu_m)) {
        return;
    }

    const float c = cosf(_show_coordinate_system.orientation_rad);
    const float s = sinf(_show_coordinate_system.orientation_rad);

    const struct log_ShowReference pkt {
        LOG_PACKET_HEADER_INIT(LOG_SHOW_REFERENCE_MSG),
        time_us      : AP_HAL::micros64(),
        show_time_us : show_time_us,
        source       : source,
        accel_ff     : static_cast<uint8_t>(is_acceleration_control_enabled() ? 1 : 0),
        ref_n        : origin_neu_m.x + (c * show_pos_mm.x + s * show_pos_mm.y) / 1000.0f,
        ref_e        : origin_neu_m.y + (s * show_pos_mm.x - c * show_pos_mm.y) / 1000.0f,
        ref_u        : origin_neu_m.z + show_pos_mm.z / 1000.0f,
        vel_n        : vel_cms.x / 100.0f,
        vel_e        : vel_cms.y / 100.0f,
        vel_u        : vel_cms.z / 100.0f,
        acc_n        : acc_cmss.x / 100.0f,
        acc_e        : acc_cmss.y / 100.0f,
        acc_u        : acc_cmss.z / 100.0f,
        n            : actual_ned_m.x,
        e            : actual_ned_m.y,
        u            : -actual_ned_m.z,
    };

    AP::logger().WriteBlock(&pkt, sizeof(pkt));
}

// Write a geofence status log entry
void AC_DroneShowManager::write_fence_status_log_message() const
{
    AC_Fence* fence = AC_Fence::get_singleton();

    const struct log_FenceStatus pkt {
        LOG_PACKET_HEADER_INIT(LOG_FENCE_STATUS_MSG),
        time_us          : AP_HAL::micros64(),
        geo_enabled      : static_cast<uint8_t>(fence ? fence->get_enabled_fences() : 0),
        geo_breaches     : static_cast<uint8_t>(fence ? fence->get_breaches() : 0),
        geo_breach_count : static_cast<uint16_t>(fence ? fence->get_breach_count() : 0),
        hard_breach_state: static_cast<uint8_t>(hard_fence.get_breach_state()),
        bubble_breach_state: static_cast<uint8_t>(bubble_fence.get_breach_state()),
        bubble_breach_count: bubble_fence.get_action_counter(),
    };

    AP::logger().WriteBlock(&pkt, sizeof(pkt));
}

// Write a drone show event log entry
void AC_DroneShowManager::write_show_event_log_message(
    const sb_event_t *event, DroneShowEventResult result) const
{
    ssize_t scene;
    float show_clock_sec;

    if (!event) {
        return;
    }

    get_scene_index_and_show_clock_within_scene(&scene, &show_clock_sec);

    const struct log_DroneShowEvent pkt {
        LOG_PACKET_HEADER_INIT(LOG_DRONE_SHOW_EVENT_MSG),
        time_us         : AP_HAL::micros64(),
        wall_clock_ms   : get_elapsed_time_since_start_msec(),
        scene           : static_cast<uint8_t>(scene),
        show_clock_ms   : static_cast<int32_t>(show_clock_sec * 1000.0f),
        type            : static_cast<uint8_t>(event->type),
        subtype         : event->subtype,
        payload         : event->payload.as_uint32,
        result          : static_cast<uint8_t>(result),
    };

    AP::logger().WriteBlock(&pkt, sizeof(pkt));
}

// Write a message containing details about a collective RTH trigger
void AC_DroneShowManager::write_crth_trigger_log_message(float rth_start_time_sec, sb_vector3_t start) const
{
    const struct log_CollectiveRTHTrigger pkt {
        LOG_PACKET_HEADER_INIT(LOG_CRTH_TRIGGER_MSG),
        time_us           : AP_HAL::micros64(),
        wall_clock_ms     : get_elapsed_time_since_start_msec(),
        rth_start_time_ms : static_cast<uint32_t>(rth_start_time_sec * 1000.0f),
        start_x           : start.x / 1000.0f,   // convert from millimeters to meters
        start_y           : start.y / 1000.0f,   // convert from millimeters to meters
        start_z           : start.z / 1000.0f,   // convert from millimeters to meters
    };
    
    AP::logger().WriteBlock(&pkt, sizeof(pkt));
}

// Write a sequence of log messages representing the state of the screenplay
void AC_DroneShowManager::write_screenplay_log_messages()
{
    size_t i, j, num_scenes, num_segments;
    sb_screenplay_scene_t* scene;
    sb_time_axis_t* time_axis;
    const sb_time_segment_t* segment;
    
    struct log_ScreenplayEntry pkt {
        LOG_PACKET_HEADER_INIT(LOG_SCREENPLAY_ENTRY_MSG),
        time_us         : AP_HAL::micros64(),
    };
    
    pkt.seq_no = _last_time_axis_config_seq_no <= 0xFF ? static_cast<uint8_t>(_last_time_axis_config_seq_no) : 0xFF;

    num_scenes = sb_screenplay_size(&_screenplay);
    for (i = 0; i < num_scenes; i++) {
        if (i > 255) {
            break;   // too many scenes
        }

        pkt.scene = static_cast<uint8_t>(i);
        
        scene = sb_screenplay_get_scene_ptr(&_screenplay, i);
        time_axis = scene ? sb_screenplay_scene_get_time_axis(scene) : nullptr;
        if (time_axis == nullptr) {
            break;
        }
        
        pkt.origin_ms = time_axis->origin_msec;
        
        num_segments = sb_time_axis_num_segments(time_axis);
        for (j = 0; j < num_segments; j++) {
            if (j > 255) {
                break;   // too many segments
            }

            pkt.index = static_cast<uint8_t>(j);
            segment = sb_time_axis_get_segment(time_axis, j);
            if (segment == nullptr) {
                break;
            }
            
            pkt.duration_ms = segment->duration_msec;
            pkt.initial_rate = segment->initial_rate;
            pkt.final_rate = segment->final_rate;

            AP::logger().WriteBlock(&pkt, sizeof(pkt));
            
            // we do not repeat the origin from the second entry onwards
            pkt.origin_ms = 0;
        }
    }
}
