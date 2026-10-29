// .gdmb shows (AxioStack contracts/gdm-format.md, ADR 0002 phase 1).
//
// A .gdmb show is loaded next to the libskybrush machinery rather than
// instead of it. Its light program and events go into the screenplay
// unchanged, and a lower-precision copy of its trajectory serves the parts of
// the show manager that need a libskybrush trajectory (takeoff and landing
// times, the time axis, the post-show logic). Flight setpoints during the
// main show come from the .gdmb file: position, velocity and acceleration
// evaluated analytically from its Bézier segments.
//
// Phase 1 flies FLAT segments at normal tilt only. Files that need the
// maneuver, reversible-thrust or high-tilt executors are refused at load.

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>

#include "AC_DroneShowManager.h"
#include "DroneShow_Constants.h"

extern const AP_HAL::HAL &hal;

// Longest piece of the libskybrush copy of the trajectory. Each piece is a
// cubic Hermite interpolant of the .gdmb curve, exact for cubic segments;
// for the quintic takeoff and landing legs 0.5 s pieces stay within about
// a millimetre.
static constexpr uint32_t SHADOW_PIECE_MAX_US = 500000;

bool AC_DroneShowManager::_load_gdmb_show_from_memory(uint8_t* data, size_t length)
{
    gdmb_diag_t diag;
    gdmb_error_t err;
    sb_screenplay_scene_t* scene = nullptr;

    err = gdmb_show_parse(&_gdmb_show, data, length, &diag);
    if (err != GDMB_OK && length >= GDMB_HEADER_LENGTH &&
        ((data[5] & GDMB_FEATURE_MANEUVER) || (data[44] & GDMB_CAP_MANEUVER))) {
        // MANEUVER is provisional in .gdmb v1 (spec §4.11), so the reader
        // refuses it by its reserved bits; say what that means
        hal.console->printf("Show file rejected: %s: maneuvers are not part of .gdmb v1\n", gdmb_error_name(err));
        gcs().send_text(MAV_SEVERITY_ERROR, "Show rejected: maneuvers not supported yet");
        return false;
    }
    if (err != GDMB_OK) {
        hal.console->printf(
            "Show file rejected: %s at byte %lu: %s\n",
            gdmb_error_name(err), static_cast<unsigned long>(diag.offset), diag.what
        );
        gcs().send_text(
            MAV_SEVERITY_ERROR, "Show rejected: %s (%s)", gdmb_error_name(err), diag.what
        );
        return false;
    }

    // Vehicle capability gate (spec §9.1): refuse every capability that this
    // firmware does not implement or that is not enabled. The reasons fit in
    // a STATUSTEXT after "Show rejected: ".
    const char* refusal = nullptr;
    if (_gdmb_show.capabilities & GDMB_CAP_MANEUVER) {
        // Unreachable while maneuvers are provisional: kept for when they are
        // adopted and the reader accepts them
        refusal = _params.maneuvers_enabled
            ? "maneuvers not supported yet"
            : "maneuvers disabled (SHOW_MNVR_EN=0)";
    } else if (
        (_gdmb_show.capabilities & GDMB_CAP_REVERSIBLE_THRUST) ||
        gdmb_show_has_inverted_flat(&_gdmb_show)
    ) {
        refusal = "inverted (sigma -1) not supported";
    } else if (_gdmb_show.capabilities & GDMB_CAP_HIGH_TILT) {
        refusal = "high-tilt not supported";
    }

    if (refusal) {
        hal.console->printf("Show file rejected: E_CAPS: %s\n", refusal);
        gcs().send_text(MAV_SEVERITY_ERROR, "Show rejected: %s", refusal);
        gdmb_show_free(&_gdmb_show);
        return false;
    }

    if (
        sb_screenplay_append_new_scene(&_screenplay, &scene) != SB_SUCCESS ||
        !_build_scene_from_gdmb_show(scene)
    ) {
        hal.console->printf("Failed to set up the screenplay of the show\n");
        sb_screenplay_clear(&_screenplay);
        gdmb_show_free(&_gdmb_show);
        return false;
    }

    // "Loaded" is announced by the caller, once the trajectory checks pass
    return true;
}

bool AC_DroneShowManager::_build_scene_from_gdmb_show(sb_screenplay_scene_t* scene)
{
    sb_trajectory_t* trajectory = nullptr;
    sb_light_program_t* light_program = nullptr;
    sb_event_list_t* event_list = nullptr;
    bool success = false;

    trajectory = sb_trajectory_new();
    if (!trajectory || !_build_trajectory_from_gdmb_show(trajectory)) {
        goto exit;
    }
    sb_screenplay_scene_set_trajectory(scene, trajectory);

    // LIGHTS is the .skyb ledctrl bytecode, unchanged (spec §4.7). The light
    // program refers to the show data buffer, which outlives the screenplay.
    if (_gdmb_show.lights) {
        light_program = sb_light_program_new();
        if (
            !light_program ||
            sb_light_program_update_from_buffer(
                light_program,
                const_cast<uint8_t*>(_gdmb_show.lights), _gdmb_show.lights_length
            ) != SB_SUCCESS
        ) {
            goto exit;
        }
        sb_screenplay_scene_set_light_program(scene, light_program);
    }

    // EVENTS are the .skyb records with microsecond timestamps (spec §4.8);
    // the event player works in milliseconds
    if (_gdmb_show.num_events > 0) {
        event_list = sb_event_list_new(_gdmb_show.num_events);
        if (!event_list) {
            goto exit;
        }
        for (uint32_t i = 0; i < _gdmb_show.num_events; i++) {
            gdmb_event_t gev;
            sb_event_t event;

            if (!gdmb_show_get_event(&_gdmb_show, i, &gev)) {
                goto exit;
            }
            event.time_msec = static_cast<uint32_t>((uint64_t(gev.t_us) + 500) / 1000);
            event.type = static_cast<sb_event_type_t>(gev.type);
            event.subtype = gev.subtype;
            event.payload.as_uint32 = gev.payload;
            if (sb_event_list_append(event_list, &event) != SB_SUCCESS) {
                goto exit;
            }
        }
        sb_screenplay_scene_set_events(scene, event_list);
    }

    // No yaw control track: yaw comes from the .gdmb file. No RTH plan: the
    // .gdmb RTH plan is validated but not executed in phase 1, and the .skyb
    // collective RTH cannot apply to a .gdmb show.
    success = true;

exit:
    SB_XDECREF(trajectory);
    SB_XDECREF(light_program);
    SB_XDECREF(event_list);
    return success;
}

bool AC_DroneShowManager::_build_trajectory_from_gdmb_show(sb_trajectory_t* trajectory)
{
    // Pieces of at most SHADOW_PIECE_MAX_US, never across a segment boundary,
    // with knots on whole milliseconds of the .gdmb timeline so that the copy
    // does not drift in time
    struct Knot {
        sb_vector3_with_yaw_t pos;   // [mm]
        sb_vector3_with_yaw_t vel;   // [mm/s]
    };

    // Joints are C2, so evaluating a knot on a segment boundary from the
    // next segment's side is as good as from the previous one's
    auto knot_at = [this](int64_t t_us, Knot& knot) -> bool {
        gdmb_state_t st;
        if (gdmb_show_evaluate(&_gdmb_show, t_us, &st) != GDMB_OK) {
            return false;
        }
        knot.pos = { float(st.p[0] * 1000), float(st.p[1] * 1000), float(st.p[2] * 1000), 0 };
        knot.vel = { float(st.v[0] * 1000), float(st.v[1] * 1000), float(st.v[2] * 1000), 0 };
        return true;
    };

    // First pass: find the largest coordinate to choose the scale of the
    // int16 libskybrush encoding
    float max_abs_mm = 0;
    for (uint32_t pass = 0; pass < 2; pass++) {
        sb_trajectory_builder_t builder;
        uint8_t scale = 1;

        if (pass == 1) {
            // floor() in the builder; scale up so that every control point fits
            scale = static_cast<uint8_t>(MIN(127.0f, ceilf((max_abs_mm + 1.0f) / 32000.0f)));
            if (max_abs_mm + 1.0f > 32000.0f * 127.0f) {
                hal.console->printf("Show trajectory too large for the show controller\n");
                return false;
            }
            if (sb_trajectory_builder_init(&builder, scale, 0) != SB_SUCCESS) {
                return false;
            }
        }

        Knot a;
        if (!knot_at(0, a)) {
            if (pass == 1) {
                sb_trajectory_builder_destroy(&builder);
            }
            return false;
        }

        // the builder floors to the encoding grid; offsetting by half a step rounds instead
        const float half_step = 0.5f * scale;
        auto shift = [half_step](sb_vector3_with_yaw_t v) {
            v.x += half_step;
            v.y += half_step;
            v.z += half_step;
            return v;
        };

        if (pass == 1 && sb_trajectory_builder_set_start_position(&builder, shift(a.pos)) != SB_SUCCESS) {
            sb_trajectory_builder_destroy(&builder);
            return false;
        }

        for (uint32_t i = 0; i < _gdmb_show.num_segments; i++) {
            const gdmb_segment_t& seg = _gdmb_show.segments[i];
            const uint64_t seg_end = uint64_t(seg.t0_us) + seg.dur_us;
            uint64_t ta = seg.t0_us;

            while (ta < seg_end) {
                uint64_t tb = MIN(seg_end, ta + SHADOW_PIECE_MAX_US);
                const uint32_t ms_a = (ta + 500) / 1000;
                const uint32_t ms_b = (tb + 500) / 1000;
                Knot b;

                if (ms_b == ms_a) {
                    // shorter than a millisecond on the grid; merge into the next piece
                    if (tb == seg_end) {
                        break;
                    }
                    ta = tb;
                    continue;
                }

                if (!knot_at(tb, b)) {
                    if (pass == 1) {
                        sb_trajectory_builder_destroy(&builder);
                    }
                    return false;
                }

                // Cubic Hermite in Bézier form over the piece
                const float T = (ms_b - ms_a) * 1.0e-3f;
                sb_vector3_with_yaw_t c1 = {
                    a.pos.x + a.vel.x * T / 3, a.pos.y + a.vel.y * T / 3, a.pos.z + a.vel.z * T / 3, 0
                };
                sb_vector3_with_yaw_t c2 = {
                    b.pos.x - b.vel.x * T / 3, b.pos.y - b.vel.y * T / 3, b.pos.z - b.vel.z * T / 3, 0
                };

                if (pass == 0) {
                    for (const sb_vector3_with_yaw_t* v : { &a.pos, &c1, &c2, &b.pos }) {
                        max_abs_mm = MAX(max_abs_mm, MAX(fabsf(v->x), MAX(fabsf(v->y), fabsf(v->z))));
                    }
                } else if (
                    sb_trajectory_builder_append_cubic_bezier(
                        &builder, shift(c1), shift(c2), shift(b.pos), ms_b - ms_a
                    ) != SB_SUCCESS
                ) {
                    sb_trajectory_builder_destroy(&builder);
                    return false;
                }

                a = b;
                ta = tb;
            }
        }

        if (pass == 1) {
            const sb_error_t retval = sb_trajectory_update_from_builder(trajectory, &builder);
            sb_trajectory_builder_destroy(&builder);
            return retval == SB_SUCCESS;
        }
    }

    return false;
}

void AC_DroneShowManager::_announce_show_identity_at_arming()
{
#if HAL_LOGGING_ENABLED
    write_show_identity_log_message();
#endif

    switch (_show_identity.format) {
    case ShowFileFormat_Gdmb: {
        const uint8_t* hash = _gdmb_show.show_hash;
        gcs().send_text(
            MAV_SEVERITY_INFO, "Armed with .gdmb show %02x%02x%02x%02x, CRC %08lx",
            hash[0], hash[1], hash[2], hash[3], static_cast<unsigned long>(_show_identity.header_crc32)
        );
        break;
    }
    case ShowFileFormat_Skyb:
        gcs().send_text(
            MAV_SEVERITY_INFO, "Armed with .skyb show, CRC %08lx",
            static_cast<unsigned long>(_show_identity.header_crc32)
        );
        break;
    default:
        break;
    }
}

void AC_DroneShowManager::_get_time_axis_rate_and_derivative(
    sb_screenplay_scene_t* scene, int32_t time_msec, float& rate, float& rate_derivative
)
{
    sb_time_axis_t* time_axis = scene ? sb_screenplay_scene_get_time_axis(scene) : nullptr;

    rate = 1.0f;
    rate_derivative = 0.0f;
    if (!time_axis) {
        return;
    }

    sb_time_axis_map_ex(time_axis, time_msec, &rate);

    // The time-axis API exposes tau_dot but not tau_ddot. Estimate the latter
    // symmetrically over 20 ms. Time-axis ramps are piecewise linear, so this
    // is exact away from their endpoints and bounded to one controller sample
    // of averaging at a segment boundary.
    constexpr int32_t RATE_DERIVATIVE_HALF_WINDOW_MSEC = 10;
    const int32_t before_msec = MAX(0, time_msec - RATE_DERIVATIVE_HALF_WINDOW_MSEC);
    const int32_t after_msec = MIN(INT32_MAX - RATE_DERIVATIVE_HALF_WINDOW_MSEC, time_msec) + RATE_DERIVATIVE_HALF_WINDOW_MSEC;
    float rate_before;
    float rate_after;
    sb_time_axis_map_ex(time_axis, before_msec, &rate_before);
    sb_time_axis_map_ex(time_axis, after_msec, &rate_after);
    rate_derivative = (rate_after - rate_before) / ((after_msec - before_msec) / 1000.0f);
}

bool AC_DroneShowManager::_get_gdmb_reference_at_seconds(float time, GdmbReference& ref)
{
    // The logged reference is only valid while an evaluation succeeds; once the
    // main show has ended (or no .gdmb show is loaded), SHOW stops reporting it
    _gdmb_reference_valid = false;

    if (!is_gdmb_show_loaded()) {
        return false;
    }

    if (!_get_raw_show_control_output_at_seconds(time)) {
        return false;
    }

    // Only the main show follows the .gdmb timeline
    sb_screenplay_scene_t* scene = sb_show_controller_get_current_scene(&_show_controller);
    if (!scene || sb_screenplay_scene_get_tag(scene) != SceneTag_MainShow) {
        return false;
    }

    const sb_control_output_time_t output_time = sb_show_controller_get_current_output_time(&_show_controller);
    float rate, rate_derivative;
    _get_time_axis_rate_and_derivative(scene, static_cast<int32_t>(output_time.time_msec), rate, rate_derivative);

    // The show controller works on whole milliseconds of wall-clock time; put
    // back the sub-millisecond part of the requested time
    double frac_usec = double(time) * 1.0e6 - double(output_time.time_msec) * 1000.0;
    if (!(frac_usec >= 0.0)) {
        frac_usec = 0.0;
    } else if (frac_usec > 1000.0) {
        frac_usec = 1000.0;
    }
    const double show_time = double(output_time.warped_time_in_scene_sec) * 1.0e6 + double(rate) * frac_usec;
    const int64_t show_time_us = show_time >= 0
        ? static_cast<int64_t>(show_time + 0.5)
        : -static_cast<int64_t>(0.5 - show_time);

    gdmb_state_t st;
    if (gdmb_show_evaluate(&_gdmb_show, show_time_us, &st) != GDMB_OK) {
        return false;
    }

    // Chain rule for p(tau(t)): dp/dt = p' tau_dot, d2p/dt2 = p'' tau_dot^2 + p' tau_ddot
    const float rate_sq = sq(rate);
    ref.show_time_us = show_time_us;
    ref.pos_m = Vector3f(st.p[0], st.p[1], st.p[2]);
    ref.vel_ms = Vector3f(st.v[0], st.v[1], st.v[2]) * rate;
    ref.acc_mss = Vector3f(st.a[0], st.a[1], st.a[2]) * rate_sq + Vector3f(st.v[0], st.v[1], st.v[2]) * rate_derivative;
    ref.yaw_rad = st.yaw;
    ref.yaw_rate_rads = st.yaw_rate * rate;

    _gdmb_reference = ref;
    _gdmb_reference_valid = true;

    return true;
}
